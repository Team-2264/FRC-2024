package frc.robot.subsystems.arm;

import java.util.OptionalDouble;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.lib.Math2264;
import frc.lib.motors.Neo;
import frc.lib.motors.NeoConfiguration;
import frc.robot.Constants;
import frc.robot.Conversions;

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

    public void rotateRelative(double offset) {
        double setPointRots = motors[0].getPosition() + (offset * Constants.Arm.shoulderRatio);

        motors[0].rotateTo(setPointRots);
    }

    public void rotateConstant(double speed) {
        motors[0].rotateAtSpeed(speed);

        desiredAngle = OptionalDouble.empty();
    }

    public void rotateTo(double angle) {
        desiredAngle = OptionalDouble.of(angle);
        Constants.Arm.shoulderFeedback.reset();
    }

    public void periodic() {

        SmartDashboard.setDefaultNumber("Shoulder kS", 0.0);
        SmartDashboard.setDefaultNumber("Shoulder kG", 0.0);
        SmartDashboard.setDefaultNumber("Shoulder kV", 0.0);
        SmartDashboard.setDefaultNumber("Shoulder kA", 0.0);
        SmartDashboard.setDefaultNumber("Shoulder kP", 0.0);
        SmartDashboard.setDefaultNumber("Shoulder kI", 0.0);
        SmartDashboard.setDefaultNumber("Shoulder kD", 0.0);

        double kS = SmartDashboard.getNumber("Shoulder kS", 0); 
        double kG = SmartDashboard.getNumber("Shoulder kG", 0); 
        double kV = SmartDashboard.getNumber("Shoulder kV", 0); 
        double kA = SmartDashboard.getNumber("Shoulder kA", 0); 
        double kP = SmartDashboard.getNumber("Shoulder kP", 0);
        double kI = SmartDashboard.getNumber("Shoulder kI", 0);
        double kD = SmartDashboard.getNumber("Shoulder kD", 0);

        Constants.Arm.shoulderFeedForward = new ArmFeedforward(
            kS,
            kG,
            kV,
            kA
        );

        Constants.Arm.shoulderFeedback.setP(kP);
        Constants.Arm.shoulderFeedback.setI(kI);
        Constants.Arm.shoulderFeedback.setD(kD);

        if(desiredAngle.isPresent()) {
            double desiredAngle = this.desiredAngle.getAsDouble();
            double currentAngle = this.getRots();

            double feedforward = Constants.Arm.shoulderFeedForward.calculate(desiredAngle * 2 * Math.PI, 0);
            double feedback = Constants.Arm.shoulderFeedback.calculate(currentAngle, desiredAngle);

            double voltage = Math2264.limitMagnitude(feedforward + feedback, Constants.Arm.shoulderMaxPower);
            motors[0].rotateAtSpeed(voltage);
        }
    }
}
