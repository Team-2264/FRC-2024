// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.FeedShooter;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.ToggleTurbo;
import frc.robot.commands.locks.LockArm;
import frc.robot.commands.locks.LockSwerve;
import frc.robot.commands.locks.UnlockArm;
import frc.robot.commands.locks.UnlockSwerve;
import frc.robot.enums.ArmState;
import frc.robot.enums.HeldButton;
import frc.robot.subsystems.Climbing;
import frc.robot.subsystems.HeldButtons;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.arm.Arm;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Map;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    public final Swerve swerve = new Swerve();
    private final Arm arm = new Arm(this);
    private final Climbing climbing = new Climbing();
    private final Leds leds = new Leds(Constants.LedStrip.pwmPort, Constants.LedStrip.numLeds, Constants.LedStrip.scaleFactor);
    private final Vision vision = new Vision();
    
    private final HeldButtons heldButtons = new HeldButtons();

    // Controllers
    private final CommandPS4Controller controller = new CommandPS4Controller(Constants.Operator.controllerPort);
    private final Joystick controller2 = new Joystick(Constants.Operator.controller2Port);

    // Second controller buttons
    private final JoystickButton manualArmUp = new JoystickButton(controller2, 8);
    private final JoystickButton manualArmDown = new JoystickButton(controller2, 7);
    
    private final JoystickButton manualClimbUp = new JoystickButton(controller2, 12);
    private final JoystickButton manualClimbDown = new JoystickButton(controller2, 11);
    
    // Autonomous
    private final SendableChooser<Command> autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Register named commands - pathplanner
        NamedCommands.registerCommand("startIntake", new InstantCommand(() -> arm.startIntake()));
        NamedCommands.registerCommand("stopIntake", new InstantCommand(() -> arm.stopIntake()));

        NamedCommands.registerCommand("spinupShooter", new InstantCommand(() -> arm.spinupShooter(0.1)));
        NamedCommands.registerCommand("stopShooter", new InstantCommand(() -> arm.stopShooter()));

        NamedCommands.registerCommand("armIntake", new InstantCommand(() -> arm.setState(ArmState.INTAKE)));
        NamedCommands.registerCommand("armHome", new InstantCommand(() -> arm.setState(ArmState.AMP)));

        // Configure the button bindings
        configureBindings();

        // Set up autonomous
        autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
        SmartDashboard.putData("Auto Mode", autoChooser);
        
    }

    /**
     * Configures all button bindings.
     */
    private void configureBindings() {
        // ======== Swerve ========
        swerve.setDefaultCommand(new TeleopSwerve(swerve, controller));
        controller.options().onTrue( new InstantCommand(() -> swerve.zeroGyro()));

        controller.share().onTrue(new ToggleTurbo(swerve));
        controller.share().onFalse(new ToggleTurbo(swerve)); 

        // ======== Shoulder ========
        controller.povUp().onTrue(new InstantCommand(() -> arm.setState(ArmState.AMP)));
        controller.povLeft().onTrue(new InstantCommand(() -> arm.setState(ArmState.START)));
        controller.povDown().onTrue(new InstantCommand(() -> arm.setState(ArmState.HOME)));
        
        // ======== Intake ========
        controller.R2().onTrue(new SequentialCommandGroup(
            new InstantCommand(() -> arm.startIntake()),
            new InstantCommand(() -> arm.setState(ArmState.INTAKE))

        ));
        controller.R2().onFalse(new SequentialCommandGroup(
            new InstantCommand(() -> arm.stopIntake()),
            new InstantCommand(() -> arm.setState(ArmState.HOME))

        ));

        controller.triangle().onTrue(new InstantCommand(() -> arm.endEffector.outtake(0.5)));
        controller.triangle().onFalse(new InstantCommand(() -> arm.endEffector.stopIntake()));

        // ======== Shooter ========
        controller.L2().onTrue(new SelectCommand<>(
            Map.ofEntries(
                Map.entry(HeldButton.CROSS, new SequentialCommandGroup( // Manual Shooting
                    new InstantCommand(() -> arm.spinupShooter(0.6)),
                    new InstantCommand(() -> arm.setState(ArmState.MANUAL_SHOOT))

                )),
                Map.entry(HeldButton.SQUARE, new SequentialCommandGroup( // Amp
                    new InstantCommand(() -> arm.spinupShooter(0.2)),
                    new InstantCommand(() -> arm.setState(ArmState.AMP))

                )),
                Map.entry(HeldButton.NONE, new SequentialCommandGroup( // Autolocking
                    new LockSwerve(swerve),
                    new LockArm(arm)
                
                ))

            ),
            heldButtons::currentHeld
            
        ));
        controller.L2().onFalse(new SequentialCommandGroup(
            new InstantCommand(() -> new UnlockSwerve(swerve)),
            new InstantCommand(() -> new UnlockArm(arm)),
            new InstantCommand(() -> arm.stopShooter()),
            new InstantCommand(() -> arm.setState(ArmState.HOME))

        ));

        controller.R1().onTrue(new FeedShooter(arm));

        // ======== Manual Climbing ========
        manualClimbUp.onTrue(new InstantCommand(() -> climbing.accend(0.6)));
        manualClimbUp.onFalse(new InstantCommand(() -> climbing.stopWinch()));

        manualClimbDown.onTrue(new InstantCommand(() -> climbing.descend(0.6)));
        manualClimbDown.onFalse(new InstantCommand(() -> climbing.stopWinch()));

        // ======== Manual Shoulder ========
        manualArmUp.onTrue(new InstantCommand(() -> arm.shoulder.rotateConstant(0.075)));
        manualArmUp.onFalse(new InstantCommand(() -> arm.shoulder.rotateConstant(0)));

        manualArmDown.onTrue(new InstantCommand(() -> arm.shoulder.rotateConstant(-0.075)));
        manualArmDown.onFalse(new InstantCommand(() -> arm.shoulder.rotateConstant(0)));
    
        // ======== Held buttons ========
        controller.cross().onTrue(new InstantCommand(() -> heldButtons.setHeld(HeldButton.CROSS)));
        controller.cross().onFalse(new InstantCommand(() -> heldButtons.setHeld(HeldButton.NONE)));

        controller.square().onTrue(new InstantCommand(() -> heldButtons.setHeld(HeldButton.SQUARE)));
        controller.square().onFalse(new InstantCommand(() -> heldButtons.setHeld(HeldButton.NONE)));

        controller.circle().onTrue(new InstantCommand(() -> heldButtons.setHeld(HeldButton.CIRCLE)));
        controller.circle().onFalse(new InstantCommand(() -> heldButtons.setHeld(HeldButton.NONE)));

    }

     /**
     * Returns the command to run in autonomous mode.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();

    }

    /**
     * Called every robot loop in both autonomous and teleop.
     */
    public void robotPeriodic() {
        Optional<EstimatedRobotPose> visionPose = vision.getEstimatedPose();

        if(visionPose.isPresent()) {
            swerve.addVisionMeasurement(visionPose.get());
            
        }

    }

    public Leds getLeds() {
        return leds;
    }

}
