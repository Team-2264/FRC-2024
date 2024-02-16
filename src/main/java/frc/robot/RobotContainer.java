// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.FeedShooter;
import frc.robot.commands.SetArmAngle;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.ToggleTurbo;
import frc.robot.enums.ArmState;
import frc.robot.subsystems.Climbing;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.arm.Arm;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    private final Swerve swerve = new Swerve();
    private final Arm arm = new Arm();
    private final Climbing climbing = new Climbing();
    private final Leds leds = new Leds(Constants.LedStrip.pwmPort, Constants.LedStrip.numLeds, Constants.LedStrip.scaleFactor);
    // private final Vision vision = new Vision();

    // Controllers
    private final CommandPS4Controller controller = new CommandPS4Controller(Constants.Operator.controllerPort);

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
        
        NamedCommands.registerCommand("arm5deg", new SetArmAngle(arm, 5));
        NamedCommands.registerCommand("arm10deg", new SetArmAngle(arm, 10));
        NamedCommands.registerCommand("arm15deg", new SetArmAngle(arm, 15));
        NamedCommands.registerCommand("arm20deg", new SetArmAngle(arm, 20));
        NamedCommands.registerCommand("arm25deg", new SetArmAngle(arm, 25));


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
        // swerve
        swerve.setDefaultCommand(new TeleopSwerve(swerve, controller));
        controller.options().onTrue( new InstantCommand(() -> swerve.zeroGyro()));

        controller.share().onTrue(new ToggleTurbo(swerve));
        controller.share().onFalse(new ToggleTurbo(swerve));

        // shoulder
        // controller.L2().onTrue(new InstantCommand(() ->  arm.shoulder.rotateConstant(0.075)));
        // controller.L2().onFalse(new InstantCommand(() -> arm.shoulder.rotateConstant(0)));

        // controller.L1().onTrue(new InstantCommand(() -> arm.shoulder.rotateConstant(-0.075)));
        // controller.L1().onFalse(new InstantCommand(() -> arm.shoulder.rotateConstant(0)));

        controller.povUp().onTrue(new InstantCommand(() -> arm.setState(ArmState.START)));
        controller.povLeft().onTrue(new InstantCommand(() -> arm.setState(ArmState.HOME)));
        controller.povDown().onTrue(new InstantCommand(() -> arm.setState(ArmState.INTAKE)));
        controller.povRight().onTrue(new SetArmAngle(arm, 0.25));

        controller.circle().onTrue(new InstantCommand(() -> arm.shoulder.rotateTo(0.125)));

        // intake
        controller.R2().onTrue(new InstantCommand(() -> arm.startIntake()));
        controller.R2().onFalse(new InstantCommand(() -> arm.stopIntake()));

        // shooter
        controller.triangle().onTrue(new InstantCommand(() -> arm.spinupShooter(0.8)));
        controller.cross().onTrue(new InstantCommand(() -> arm.stopShooter()));

        controller.square().onTrue(new FeedShooter(arm));

        // climbing
        controller.L2().onTrue(new InstantCommand(() -> climbing.descend(0.5)));
        controller.L2().onFalse(new InstantCommand(() -> climbing.stopWinch()));

        controller.L1().onTrue(new InstantCommand(() -> climbing.accend(0.5)));
        controller.L1().onFalse(new InstantCommand(() -> climbing.stopWinch()));

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
        // Optional<EstimatedRobotPose> visionPose = vision.getEstimatedPose();

        // if(visionPose.isPresent()) {
        //     swerve.addVisionMeasurement(visionPose.get());
        // }

    }

    public Leds getLeds() {
        return leds;
    }

}
