package frc.robot.subsystems.arm;

import java.util.OptionalDouble;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.lib.motors.Neo;
import frc.lib.motors.NeoConfiguration;
import frc.robot.Constants;
import frc.robot.Conversions;

public class Shoulder {
    private Neo[] motors;
    public DutyCycleEncoder absEncoder;

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

    public void rotateRelative(double offset) {
        double setPointRots = motors[0].getPosition() + (offset * Constants.Arm.shoulderRatio);

        motors[0].rotateTo(setPointRots);
    }

    public void rotateConstant(double speed) {
        motors[0].rotateAtSpeed(speed);

    }

}
