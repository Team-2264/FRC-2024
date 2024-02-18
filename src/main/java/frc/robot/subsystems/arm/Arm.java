package frc.robot.subsystems.arm;

import java.util.Optional;
import java.util.OptionalDouble;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.FieldPose;
import frc.lib.TrajectoryParameters;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.enums.ArmState;
import frc.robot.enums.IntakeStatus;

/**
 * Subystem for controlling the arm.
 * 
 */
public class Arm extends SubsystemBase {
    public final Shoulder shoulder;
    public final EndEffector endEffector;

    public RobotContainer container;
    
    private ArmState state;

    private Optional<Translation2d> lockedOnto = Optional.empty();

    
    private final Translation2d speakerTranslation = Constants.Targeting.speakerPose.getWpiBlue().getTranslation().toTranslation2d();



    /**
     * Constructs a new Arm instance.
     */
    public Arm(RobotContainer container) {
        shoulder = new Shoulder(Constants.Arm.neoConfigs);
        endEffector = new EndEffector(Constants.EndEffector.intakeNeoConfig, 
            Constants.EndEffector.shooterNeoConfigs,
            Constants.EndEffector.beamBreakPorts);

        state = ArmState.START;

        this.container = container;
        
    }

    /**
     * Sets the state of the arm. This is the main way to control the arm.
     * 
     * @param state
     */
    public void setState(ArmState state) {
        this.state = state;

        if (state != ArmState.CUSTOM) {
            setShoulderAngle(state.shoulderAngle());

        }

    }

    public void lockOnto(Translation2d translation2d) {
        lockedOnto = Optional.of(translation2d);
    }

    public void unlock() {
        lockedOnto = Optional.empty();
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
     * Spins up the shooter motors to a given speed.
     * Speed is a value between -1 and 1.
     * 
     * @param speed The speed to spin the motors at.
     */
    public void spinupShooter(double speed) {
        endEffector.spinupShooter(speed);

    }
    
    /**
     * Stops the shooter motors.
     */
    public void stopShooter() {
        endEffector.stopShooter();
    
    }

    /**
     * Starts the intake.
     */
    public void startIntake() {
        if (endEffector.intakeStatus() == IntakeStatus.STOPPED && !endEffector.hasNote()) {
            endEffector.intake(Constants.EndEffector.intakeSpeed);

        }

    }
    
    /**
     * Stops the intake.
     */
    public void stopIntake(){
        endEffector.stopIntake();
    }

    @Override 
    public void periodic() {
        shoulder.periodic();

        SmartDashboard.putString("ENDEFFECTOR TEST", endEffector.intakeStatus().toString());

        SmartDashboard.putNumber("Shoulder abs encoder", shoulder.absEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("Shoulder rel rots", shoulder.getRots());
        SmartDashboard.putBoolean("intake BEAMS", endEffector.hasNote());

        // Stop intaking if we have a note
        if (endEffector.intakeStatus() == IntakeStatus.INTAKING && endEffector.hasNote()) {
            endEffector.stopIntake();

        }

        // If we are locked onto a speaker, calculate the arm angle
        if (lockedOnto.isPresent()) {
            final double distance_to_speaker = container.swerve.getPose().getTranslation().minus(speakerTranslation).getNorm();

            double heightOffset = (Math.pow(((0.7 * distance_to_speaker) - 0.9), 3));

            Constants.Targeting.speakerPose =  FieldPose.fromWpiBlue(new Pose3d(
            new Translation3d(
                Units.inchesToMeters(8), // x
                Units.inchesToMeters(218.42), // y
                Units.inchesToMeters(82.9 + heightOffset)), // z
                new Rotation3d()
            ));

            Constants.Targeting.armParameters = new TrajectoryParameters()
                .withArmLength(0.5917)
                .withGoalHeight(Constants.Targeting.speakerPose.getWpiBlue().getZ() - Constants.Targeting.pivotToGround)
                .withLaunchAngleOffset(107.258 * (Math.PI/180.0));

            final OptionalDouble angle_estimate = Constants.Targeting.getSpeakerArmAngle(distance_to_speaker);
            if(angle_estimate.isPresent()) {
                setShoulderAngle(angle_estimate.getAsDouble());
            }

        }
           
    }

}
