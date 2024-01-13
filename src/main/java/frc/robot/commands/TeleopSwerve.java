// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A command for teleoperated control of a swerve drive subsystem using a PS4 controller.
 */
public class TeleopSwerve extends Command {
    private final Swerve swerve;

    private final CommandPS4Controller controller;

    /**
     * Creates a new TeleopSwerve command.
     *
     * @param swerve The swerve drive subsystem to control.
     * @param controller The PS4 controller used for input.
     */
    public TeleopSwerve(Swerve swerve, CommandPS4Controller controller) {
        this.swerve = swerve;
        addRequirements(swerve);

        this.controller = controller;

    }

    /**
     * Applies a polynomial curve to the input value to modify sensitivity.
     *
     * @param input The input value to be curved.
     * @return The curved output value.
     */
    public double xyCurve(double input) {
        return (0.5 * input) + (0.2 * Math.pow(input, 3)) + (0.25 * (Math.pow(input, 5)));

    }

    /**
     * Returns the input value without any modification.
     *
     * @param input The input value.
     * @return The same input value.
     */
    public double rCurve(double input) {
        return input;

    }

    /**
     * Called repeatedly while the command is scheduled.
     * Reads controller input, applies deadbands and curves, and drives the swerve subsystem.
     */
    @Override
    public void execute() {
        double yAxis = -controller.getLeftY();
        double xAxis = -controller.getLeftX();
        double rAxis = -controller.getRightX();

        xAxis = (Math.abs(xAxis) < Constants.Operator.stickDeadband) ? 0 : xAxis;
        yAxis = (Math.abs(yAxis) < Constants.Operator.stickDeadband) ? 0 : yAxis;
        rAxis = (Math.abs(rAxis) < Constants.Operator.stickDeadband) ? 0 : rAxis;

        xAxis = xyCurve(xAxis);
        yAxis = xyCurve(yAxis);
        rAxis = rCurve(rAxis);
        
        Translation2d translation = new Translation2d(yAxis, xAxis).times(Constants.Swerve.maxSpeed);
        double rotation = rAxis * Constants.Swerve.maxAngularVelocity;

        SmartDashboard.putNumber("Rot", rotation);
        SmartDashboard.putString("translation", translation.toString());
    
        swerve.drive(translation, rotation);

    }

    /**
     * Indicates whether the command has finished its task.
     *
     * @return Always returns false, indicating that the command is never finished.
     */
    @Override
    public boolean isFinished() {
        return false;

    }

}
