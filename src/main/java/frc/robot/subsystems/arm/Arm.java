package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.enums.ArmState;
import frc.robot.enums.ShoulderPosition;

/**
 * Subystem for controlling the arm.
 * 
 */
public class Arm extends SubsystemBase {
    private final Shoulder shoulder;
    private final EndEffector endEffector;
    
    private ArmState state;

    /**
     * Constructs a new Arm instance.
     */
    public Arm() {
        shoulder = new Shoulder();
        endEffector = new EndEffector(Constants.EndEffector.intakeNeoConfig, 
            Constants.EndEffector.shooterNeoConfigs,
            Constants.EndEffector.beamBreakPort);

        state = ArmState.START;
        
    }

    /**
     * Spins up the shooter motors to a given speed.
     * Speed is a value between -1 and 1.
     * 
     * @param speed The speed to spin the motors at.
     */
    public void spinupShooter(double speed) {
        endEffector.spinupShooter(speed);

    }
    
    /**
     * Stops the shooter motors.
     */
    public void stopShooter() {
        endEffector.stopShooter();
    
    }

    // Shoulder setpoints
    public void shoulderToBottom(){
        shoulder.goToPosition(ShoulderPosition.Bottom);
        // call method for getting intakes
    }

    public void shoulderToMiddle(){
        shoulder.goToPosition(ShoulderPosition.Middle);
    }

    public void shoulderToTop(){
        shoulder.goToPosition(ShoulderPosition.Top);

    }

    /**
     * Starts the intake.
     */
    public void startIntake(){
        endEffector.startIntake();
    }

    /**
     * Stops the intake.
     */
    public void stopIntake(){
        endEffector.stopIntake();
    }

    @Override 
    public void periodic() {
        // Stop intaking if the end effector has a note
        if (endEffector.intaking() && endEffector.hasNote()) {
            endEffector.stopIntake();

        }
        
    }

}
