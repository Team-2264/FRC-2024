package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.arm.EndEffector;
import frc.robot.arm.Shoulder;
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
        endEffector = new EndEffector(Constants.EndEffector.intakeMotorID, 
            Constants.EndEffector.shooterNeoConfigs,
            Constants.EndEffector.beamBreakPort);

        state = ArmState.HOME;
        
    }

    /**
     * Sets the state of the arm.
     * 
     * @param status The new state of the arm.
     */
    public void setState(ArmState state) {
        this.state = state;
        
    }

    /**
     * Returns the state of the arm.
     * 
     * @return The state of the arm.
     */
    public ArmState getState() {
        return state;
        
    }
  
    // Shoulder setpoints
    public void ShoulderToBottom(){
        shoulder.goToPosition(ShoulderPosition.Bottom);
        // call method for getting intakes
    }

    public void ShoulderToMiddle(){
        shoulder.goToPosition(ShoulderPosition.Middle);
    }

    public void ShoulderToTop(){
        shoulder.goToPosition(ShoulderPosition.Top);

    }

    /**
     * Starts the intake.
     */
    public void Intake(){
        endEffector.startIntake();
    }

    /**
     * Stops the intake.
     */
    public void StopIntake(){
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
