package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.lib.motors.Neo;
import frc.lib.motors.NeoConfiguration;
import frc.robot.Constants;
import frc.robot.Conversions;

public class Shoulder {
    private Neo[] motors;
    private DutyCycleEncoder absEncoder;

    /**
     * Creates a new Shoulder object.
     * 
     * @param motorConfigs The configurations for the motors on the shoulder.
     */
    public Shoulder(NeoConfiguration[] neoConfigs) {
        // create encoder
        absEncoder = new DutyCycleEncoder(Constants.Arm.encoderID);
        

        // create motors
        motors = new Neo[neoConfigs.length];
        for(int i = 0; i < neoConfigs.length; i++) {
            motors[i] = new Neo(neoConfigs[i]);

        }

        // Bottom the shoulder on startup
        rotateTo(15);
                
    }

    /**
     * Gets the angle of the shoulder relative to its bottom position.
     * The bottom position is defined as the position where the arm is parallel to the ground.
     * 
     * @return The angle of the shoulder in degrees.
     */
    private double getAngle() {
        double absAngle = absEncoder.getAbsolutePosition();
        double relAngle = absAngle + Constants.Arm.shoulderOffset;
        
        return relAngle;
    }

    /**
     * Rotates the shoulder to a target angle relative to its bottom position.
     * The bottom position is defined as the position where the arm is parallel to the ground.
     * 
     * @param targetAngle The angle to rotate to in degrees.
     */
    public void rotateTo(int targetAngle) {
        double currentAngle = getAngle();
        double angleDifference = targetAngle - currentAngle;

        for (int i = 0; i < motors.length; i++) {
            motors[i].rotateTo(Conversions.degreesToRevs(angleDifference, Constants.Arm.shoulderRatio));

        }

    }

}
