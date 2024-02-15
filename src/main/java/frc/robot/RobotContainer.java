// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.MoveArmBy;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.ToggleTurbo;
import frc.robot.enums.ArmState;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.arm.Arm;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.pathplanner.lib.auto.AutoBuilder;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private Swerve swerve = new Swerve();
    private final Arm arm = new Arm();
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
        configureBindings();

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

        controller.square().onTrue(new ToggleTurbo(swerve));
        controller.square().onFalse(new ToggleTurbo(swerve));

        // arm general
        controller.povDown().onTrue(new MoveArmBy(arm, -1.0/360.0));
        // controller.povLeft().onTrue(new InstantCommand(() -> arm.setState(ArmState.HOME)));
        controller.povUp().onTrue(new MoveArmBy(arm, 1.0/360.0));

        // // arm: intake
        controller.R2().onTrue(new InstantCommand(() -> arm.startIntake()));
        controller.R2().onFalse(new InstantCommand(() -> arm.stopIntake()));

        // // arm: shooter
        controller.triangle().onTrue(new InstantCommand(() -> arm.spinupShooter(0.4)));
        controller.cross().onTrue(new InstantCommand(() -> arm.stopShooter()));

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
