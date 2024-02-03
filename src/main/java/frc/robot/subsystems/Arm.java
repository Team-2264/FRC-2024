package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.EndEffector;
import frc.robot.Shoulder;
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
        endEffector = new EndEffector();

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
  
    // set shoulde position to bottm + get intake
    public void SetIntake(){
        shoulder.goToPosition(ShoulderPosition.Bottom);
        // call method for getting intakes
    }

    public void ShoulderToMiddle(){
        shoulder.goToPosition(ShoulderPosition.Middle);
    }

    public void ShoulderToTop(){
        shoulder.goToPosition(ShoulderPosition.Top);

    }

    @Override 
    public void periodic() {
        

    }

}
