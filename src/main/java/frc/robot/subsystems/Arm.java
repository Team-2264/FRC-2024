package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.EndEffector;
import frc.robot.Shoulder;
import frc.robot.enums.ArmStatus;

/**
 * Subystem for controlling the arm.
 * 
 */
public class Arm extends SubsystemBase {
    private final Shoulder shoulder;
    private final EndEffector endEffector;
    
    private ArmStatus status;

    /**
     * Constructs a new Arm instance.
     */
    public Arm() {
        shoulder = new Shoulder();
        endEffector = new EndEffector();

        status = ArmStatus.HOME;
        
    }

    /**
     * Sets the state of the arm.
     * 
     * @param status The new state of the arm.
     */
    public void setState(ArmStatus status) {
        this.status = status;
        
    }

    @Override 
    public void periodic() {
        

    }
    
}
