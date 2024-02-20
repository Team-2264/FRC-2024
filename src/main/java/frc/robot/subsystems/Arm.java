package frc.robot.subsystems;

import java.util.Optional;
import java.util.OptionalDouble;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.TrajectoryParameters;
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
            setShoulderAngle(state.shoulderAngle());

        }

    }

    public void lockOnto(Translation2d translation2d) {
        lockedOnto = Optional.of(translation2d);
    }

    public void unlock() {
        lockedOnto = Optional.empty();
    }

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

    public ArmState getState() {
        return armState;
    }

    @Override 
    public void periodic() {
        shoulder.periodic();

        SmartDashboard.putString("Arm Status", armState.toString());

        // If we are locked onto a speaker, calculate the arm angle
        if (lockedOnto.isPresent()) {
            final double distance_to_speaker = container.swerve.getPose().getTranslation().minus(lockedOnto.get()).getNorm();

            Optional<Alliance> alliance2 = DriverStation.getAlliance();
            if(alliance2.isEmpty()) {
                throw new RuntimeException("Failed to get native pose: Alliance is not set in Driverstation");
            }

            Alliance alliance = alliance2.get();
            if(alliance == Alliance.Blue) {
                Constants.Targeting.armParameters = new TrajectoryParameters()
                    .withArmLength(0.5917)
                    .withGoalHeight(Constants.Targeting.blueSpeakerPose.getWpiBlue().getZ() - Constants.Targeting.pivotToGround)
                    .withLaunchAngleOffset(107.258 * (Math.PI/180.0));

            } else {
                Constants.Targeting.armParameters = new TrajectoryParameters()
                    .withArmLength(0.5917)
                    .withGoalHeight(Constants.Targeting.redSpeakerPose.getWpiBlue().getZ() - Constants.Targeting.pivotToGround)
                    .withLaunchAngleOffset(107.258 * (Math.PI/180.0));

            }   

            final OptionalDouble angle_estimate = Constants.Targeting.getSpeakerArmAngle(distance_to_speaker);
            if(angle_estimate.isPresent()) {
                setShoulderAngle(angle_estimate.getAsDouble());
            }

        }
           
    }

}
