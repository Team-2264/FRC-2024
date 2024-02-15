package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.motors.Neo;
import frc.robot.Constants;
import frc.robot.enums.ClimbingStatus;

/**
 * Subsystem for climbing.
 */
public class Climbing extends SubsystemBase {
    private Neo winchMotor;
    private ClimbingStatus climbingStatus;
    

    /**
     * Creates a new Climbing object.
     */
    public Climbing() {
        winchMotor = new Neo(Constants.Climbing.winchNeoConfig);
        climbingStatus = ClimbingStatus.STOPPED;

    }

    /**
     * Accends the winch.
     * 
     * @param speed The speed to accend at from 0 to 1.
     */
    public void accend(double speed) {
        winchMotor.rotateAtSpeed(speed);
        climbingStatus = ClimbingStatus.ACCENDING;

    }

    /**
     * Decends the winch.
     * 
     * @param speed The speed to decend at from 0 to 1.
     */
    public void descend(double speed) {
        winchMotor.rotateAtSpeed(-speed);
        climbingStatus = ClimbingStatus.DESCENDING;

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
