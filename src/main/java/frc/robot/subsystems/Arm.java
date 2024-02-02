package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.EndEffector;
import frc.robot.Shoulder;

/**
 * Subystem for controlling the arm.
 * 
 */
public class Arm extends SubsystemBase {
    private final Shoulder shoulder;
    private final EndEffector endEffector;

    /**
     * Constructs a new Arm instance.
     */
    public Arm() {
        shoulder = new Shoulder();
        endEffector = new EndEffector();
        
    }

    @Override 
    public void periodic() {
        

    }
    
}
