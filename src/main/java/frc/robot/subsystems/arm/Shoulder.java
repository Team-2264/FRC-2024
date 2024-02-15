package frc.robot.subsystems.arm;

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
        absEncoder = new DutyCycleEncoder(Constants.Arm.angleEncoderID);
        

        // create motors
        motors = new Neo[neoConfigs.length];
        for(int i = 0; i < neoConfigs.length; i++) {
            motors[i] = new Neo(neoConfigs[i]);

        }
                
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

    public void rotateRelative(double offset) {
        motors[0].rotateTo(motors[0].getPosition() + offset);
    }

    /**
     * Rotates the shoulder to a target angle relative to its bottom position.
     * The bottom position is defined as the position where the arm is parallel to the ground.
     * 
     * @param targetAngle The angle to rotate to in degrees.
     */
    public void rotateTo(double targetAngle) {
        double currentAngle = getAngle();
        double angleDifference = Conversions.degreesToRevs(targetAngle - currentAngle, 1);
        double targetNativeAngle = angleDifference + motors[0].getPosition();

        motors[0].rotateTo(Conversions.degreesToRevs(targetNativeAngle, Constants.Arm.shoulderRatio));

    }

}
