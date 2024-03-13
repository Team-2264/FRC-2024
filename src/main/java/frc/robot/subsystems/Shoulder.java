package frc.robot.subsystems;

import java.util.OptionalDouble;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.lib.Math2264;
import frc.lib.motors.Neo;
import frc.lib.motors.NeoConfiguration;
import frc.robot.Constants;

/**
 * Subystem for controlling the shoulder.
 * 
 */
public class Shoulder {
    private Neo[] motors;
    public DutyCycleEncoder absEncoder;

    private OptionalDouble desiredAngle = OptionalDouble.empty();

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
    public double getRots() {
        double absRotations = absEncoder.getAbsolutePosition();
        double relRotations = -1.0 * (absRotations - Constants.Arm.shoulderOffset);
        
        return relRotations;
    }

    /**
     * Rotates the shoulder at a constant speed.
     * @param speed The speed to rotate the shoulder at.
     * Reset the desiredangle after rotating.
     */
    public void rotateConstant(double speed) {
        motors[0].rotateAtSpeed(speed);

        desiredAngle = OptionalDouble.empty();
    }

    /**
     * Sets the angle that we aim to rotate to.
     * Does not trigger actual rotation.
     * Angle in rotations (not degrees)
     * @param angle The angle that we aim to rotate to.
     */
    public void rotateTo(double angle) {
        if(desiredAngle.isEmpty()) {
            Constants.Arm.shoulderFeedback.reset();
        }

        desiredAngle = OptionalDouble.of(angle);
    }

    public void periodic() {
        double currentAngle = this.getRots();

        // limit the shoulder going beyond the limit angle.
        if(currentAngle > Constants.Arm.shoulderMaxAngle) {
            motors[0].stop();
            return;
        }

        if(desiredAngle.isPresent()) {
            double desiredAngle = this.desiredAngle.getAsDouble();

            double feedforward = Constants.Arm.shoulderFeedForward.calculate(desiredAngle * 2 * Math.PI, 0);
            double feedback = Constants.Arm.shoulderFeedback.calculate(currentAngle, desiredAngle);
            double voltage = Math2264.limitMagnitude(feedforward + feedback, Constants.Arm.shoulderMaxPower);
            motors[0].rotateAtSpeed(voltage);
            
        }
    }
}
