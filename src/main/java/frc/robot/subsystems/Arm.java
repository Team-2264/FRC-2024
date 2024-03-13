package frc.robot.subsystems;

import java.util.Optional;
import java.util.OptionalDouble;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.enums.ArmState;

/**
 * Subystem for controlling the arm.
 * 
 */
public class Arm extends SubsystemBase {
    public final Shoulder shoulder;

    public RobotContainer container;
    
    private ArmState armState;

    private Optional<Translation2d> lockedOnto = Optional.empty();

    /**
     * Constructs a new Arm instance.
     */
    public Arm(RobotContainer container) {
        shoulder = new Shoulder(Constants.Arm.shoulderNeoConfigs);
      
        armState = ArmState.START;

        this.container = container;
        
    }

    /**
     * Sets the state of the arm. This is the main way to control the arm.
     * 
     * @param state
     */
    public void setState(ArmState state) {
        this.armState = state;

        if (state != ArmState.LOCKED) {
            unlock();
            setShoulderAngle(state.shoulderAngle());

        }

    }

    /**
     * automatically sets the arm position based on the distance from the speaker
     * @param translation2d
     */
    public void lockOnto(Translation2d translation2d) {
        lockedOnto = Optional.of(translation2d);
    }

    /**
     * allows arm to move based on ArmState(enum)
     */
    public void unlock() {
        lockedOnto = Optional.empty();
    }

    /**
     * check if the arm is locked
     * @return if the arm is locked
     */
    public boolean locked() {
        return lockedOnto.isPresent();

    }
    
    /**
     * Rotates the shoulder to a given angle.
     * 
     * @param angle The angle to rotate the shoulder to.
     */
    public void setShoulderAngle(double angle) {
        shoulder.rotateTo(angle);

    }
    
    /**
     * 
     * @return armState enum
     */
    public ArmState getState() {
        return armState;
    }

    @Override 
    public void periodic() {
        shoulder.periodic();

        SmartDashboard.putString("Arm Status", armState.toString());

        // If we are aiming to the speaker, calculate the arm angle
        if (lockedOnto.isPresent()) {
            final double distance_to_speaker = container.swerve.getPose().getTranslation().minus(lockedOnto.get()).getNorm();

            final OptionalDouble angle_estimate = Constants.Targeting.getSpeakerArmAngle(distance_to_speaker);
            SmartDashboard.putString("est angle", angle_estimate.toString());
            
            // limits angle within 0~0.25 boundary
            if(angle_estimate.isPresent()) {
                if(angle_estimate.getAsDouble() > 0.25) {
                    return;
                } else if(angle_estimate.getAsDouble() < 0) {
                    return;
                }
                setShoulderAngle(angle_estimate.getAsDouble());
            }

        }
           
    }

}
