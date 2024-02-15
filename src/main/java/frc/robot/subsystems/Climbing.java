package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.motors.Neo;
import frc.robot.Constants;

/**
 * Subsystem for climbing.
 */
public class Climbing extends SubsystemBase {
    Neo winchMotor;

    /**
     * Creates a new Climbing object.
     */
    public Climbing() {
        winchMotor = new Neo(Constants.Climbing.winchNeoConfig);

    }

    /**
     * Spins the winch motor at a given speed.
     * 
     * @param speed The speed to spin the winch motor at.
     */
    public void spinWinch(double speed) {
        winchMotor.rotateAtSpeed(speed);
    }

    /**
     * Stops the winch motor.
     */
    public void stopWinch() {
        winchMotor.stop();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    
}
