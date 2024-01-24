// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;

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
    private final Limelight limelight = new Limelight();
    private final Leds leds = new Leds(Constants.LedStrip.pwmPort, Constants.LedStrip.numLeds, Constants.LedStrip.scaleFactor);

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
        swerve.setDefaultCommand(new TeleopSwerve(swerve, controller));

        controller.options().onTrue( new InstantCommand(() -> swerve.zeroGyro()));

    }

    public void resetEncoders() {
        swerve.resetEncoders();

    }

     /**
     * Returns the command to run in autonomous mode.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();

    }

    public Leds getLeds() {
        return leds;
    }

}
