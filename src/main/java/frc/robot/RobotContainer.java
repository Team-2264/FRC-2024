// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.cmdGroups.ResetHome;
import frc.robot.commands.FeedShooter;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.ToggleTurbo;
import frc.robot.commands.locks.LockArm;
import frc.robot.commands.locks.LockShooter;
import frc.robot.commands.locks.LockSwerve;
import frc.robot.commands.locks.UnlockArm;
import frc.robot.commands.locks.UnlockShooter;
import frc.robot.commands.locks.UnlockSwerve;
import frc.robot.enums.ArmState;
import frc.robot.enums.HeldButton;
import frc.robot.enums.IntakeStatus;
import frc.robot.enums.ShooterStatus;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climbing;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.HeldButtons;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
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
    public final Arm arm = new Arm(this);
    public final EndEffector endEffector = new EndEffector(this);

    private final Climbing climbing = new Climbing();
    public final Leds leds = new Leds(Constants.LedStrip.pwmPort, Constants.LedStrip.numLeds, Constants.LedStrip.scaleFactor);
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

    private final JoystickButton manualIntake = new JoystickButton(controller2, 10);
    private final JoystickButton manualOuttake = new JoystickButton(controller2, 9);

    private final JoystickButton manualRev = new JoystickButton(controller2, 1);
    
    // Autonomous
    private final SendableChooser<Command> autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Register named commands - pathplanner
        NamedCommands.registerCommand("intake", new SequentialCommandGroup(
            new InstantCommand(() -> endEffector.intake(0.6)),
            new InstantCommand(() -> arm.setState(ArmState.INTAKE))

        ));
        NamedCommands.registerCommand("home", new SequentialCommandGroup(
            new InstantCommand(() -> endEffector.stopIntake()),
            new InstantCommand(() -> arm.setState(ArmState.HOME))

        )); 
        NamedCommands.registerCommand("lockArm", new LockArm(arm));
        NamedCommands.registerCommand("lockSwerve", new LockSwerve(swerve));
        NamedCommands.registerCommand("lockShooter", new LockShooter(endEffector));

        NamedCommands.registerCommand("unlockArm", new UnlockArm(arm));
        NamedCommands.registerCommand("unlockSwerve", new UnlockSwerve(swerve));
        NamedCommands.registerCommand("unlockShooter", new UnlockShooter(endEffector));
        
        NamedCommands.registerCommand("feedShooter", new FeedShooter(endEffector, 1));

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
        controller.R2().onTrue(new ConditionalCommand(
            new SequentialCommandGroup(
                new InstantCommand(() -> endEffector.intake(0.6)),
                new InstantCommand(() -> arm.setState(ArmState.INTAKE))
            ),
            new InstantCommand(),
            () -> (!endEffector.hasNote() && endEffector.intakeStatus() == IntakeStatus.STOPPED && arm.getState() == ArmState.HOME)

        ));
        controller.R2().onFalse(new ConditionalCommand(
            new SequentialCommandGroup(
                new InstantCommand(() -> endEffector.stopIntake()),
                new InstantCommand(() -> arm.setState(ArmState.HOME))
            ),
            new InstantCommand(),
            () -> (arm.getState() == ArmState.INTAKE)

        ));
        
        controller.triangle().onTrue(new SequentialCommandGroup(
            new InstantCommand(() -> endEffector.outtake(0.4)),
            new InstantCommand(() -> endEffector.spinupShooter(-1))
        ));
        controller.triangle().onFalse(new SequentialCommandGroup(
            new InstantCommand(() -> endEffector.stopIntake()),
            new InstantCommand(() -> endEffector.stopShooter())
        ));

        // ======== Shooter ========
        controller.L2().onTrue(new ConditionalCommand(
            new SelectCommand<>(
                Map.ofEntries(
                    Map.entry(HeldButton.CROSS, new SequentialCommandGroup( // Manual Shooting
                        new InstantCommand(() -> endEffector.spinupShooter(0.6)),
                        new InstantCommand(() -> arm.setState(ArmState.MANUAL_SHOOT))

                    )),
                    Map.entry(HeldButton.SQUARE, new SequentialCommandGroup( // Amp
                        new InstantCommand(() -> arm.setState(ArmState.AMP))

                    )),
                    Map.entry(HeldButton.NONE, new SequentialCommandGroup( // Autolocking
                        new LockSwerve(swerve),     
                        new LockArm(arm),
                        new LockShooter(endEffector)

                    ))

                ),
                heldButtons::currentHeld
            
            ),
            new ResetHome(arm, swerve, endEffector),
            () -> (arm.getState() == ArmState.HOME)

        ));

        controller.R1().onTrue(new ConditionalCommand(
            new SequentialCommandGroup(
                new ConditionalCommand(
                    new SequentialCommandGroup(
                        new InstantCommand(() -> endEffector.manualIntake(0.9)),
                        new InstantCommand(() -> endEffector.manualShooter(0.9))
                    ),
                    new SequentialCommandGroup(
                        new FeedShooter(endEffector, 1),
                        new ResetHome(arm, swerve, endEffector)
                    ),
                    () -> (arm.getState() == ArmState.AMP)

                )

            ),
            new InstantCommand(),
            () -> (endEffector.shooterStatus() == ShooterStatus.SPINNING || endEffector.shooterStatus() == ShooterStatus.LOCKED || arm.getState() == ArmState.AMP)
            
        ));

        controller.R1().onFalse(new ConditionalCommand(
            new SequentialCommandGroup(
                new InstantCommand(() -> endEffector.stopIntake()),
                new InstantCommand(() -> endEffector.stopShooter()),
                new ResetHome(arm, swerve, endEffector)
            ),
            new InstantCommand(),
            () -> (arm.getState() == ArmState.AMP)

        ));

        // ======== Manual Climbing ========
        manualClimbUp.onTrue(new InstantCommand(() -> climbing.accend(1)));
        manualClimbUp.onFalse(new InstantCommand(() -> climbing.stopWinch()));

        manualClimbDown.onTrue(new InstantCommand(() -> climbing.descend(1)));
        manualClimbDown.onFalse(new InstantCommand(() -> climbing.stopWinch()));

        // ======== Manual Intake ========
        manualIntake.onTrue(new SequentialCommandGroup(
            new InstantCommand(() -> endEffector.manualIntake(1)),
            new InstantCommand(() -> endEffector.manualShooter(1))
        ));
        manualIntake.onFalse(new SequentialCommandGroup(
            new InstantCommand(() -> endEffector.stopIntake()),
            new InstantCommand(() -> endEffector.stopShooter())
        ));

        manualOuttake.onTrue(new SequentialCommandGroup(
            new InstantCommand(() -> endEffector.manualIntake(-1)),
            new InstantCommand(() -> endEffector.manualShooter(-1))
        ));
        manualOuttake.onFalse(new SequentialCommandGroup(
            new InstantCommand(() -> endEffector.stopIntake()),
            new InstantCommand(() -> endEffector.stopShooter())
        ));

        // ======== Manual Shoulder ========
        manualArmUp.onTrue(new InstantCommand(() -> arm.shoulder.rotateConstant(0.075)));
        manualArmUp.onFalse(new InstantCommand(() -> arm.shoulder.rotateConstant(0)));

        manualArmDown.onTrue(new InstantCommand(() -> arm.shoulder.rotateConstant(-0.075)));
        manualArmDown.onFalse(new InstantCommand(() -> arm.shoulder.rotateConstant(0)));

        // ======== Manual Shooter ========
        manualRev.onTrue(new LockShooter(endEffector));
    
        // ======== Held buttons ========
        controller.cross().onTrue(new InstantCommand(() -> heldButtons.setHeld(HeldButton.CROSS)));
        controller.cross().onFalse(new InstantCommand(() -> heldButtons.setHeld(HeldButton.NONE)));

        controller.square().onTrue(new InstantCommand(() -> heldButtons.setHeld(HeldButton.SQUARE)));
        controller.square().onFalse(new InstantCommand(() -> heldButtons.setHeld(HeldButton.NONE)));

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
