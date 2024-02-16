package frc.robot.subsystems.arm;

import java.util.OptionalDouble;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.lib.Math2264;
import frc.lib.motors.Neo;
import frc.lib.motors.NeoConfiguration;
import frc.robot.Constants;

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
     */
    public void rotateConstant(double speed) {
        motors[0].rotateAtSpeed(speed);

        desiredAngle = OptionalDouble.empty();
    }

    public void rotateTo(double angle) {
        desiredAngle = OptionalDouble.of(angle);
        Constants.Arm.shoulderFeedback.reset();
    }

    public void periodic() {
        double currentAngle = this.getRots();

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
