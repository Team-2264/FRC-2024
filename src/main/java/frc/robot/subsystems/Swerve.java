package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.configs.Pigeon2Configuration;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The Swerve class represents a swerve drive subsystem for a robot.
 * This class handles swerve modules, odometry, and gyro integration.
 */
public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public Pigeon2 pidgey;

    public SwerveModule[] swerveModules;

    private boolean fieldRelative;

    /**
     * Constructs a Swerve subsystem instance.
     */
    public Swerve() {
        pidgey = new Pigeon2(Constants.Swerve.pigeonID);
        pidgey.getConfigurator().apply(new Pigeon2Configuration());
        zeroGyro();

        swerveModules = new SwerveModule[] {
            new SwerveModule(0,
                Constants.Swerve.Mod0.driveMotorID,
                Constants.Swerve.Mod0.angleMotorID,
                Constants.Swerve.Mod0.angleEncoderID,
                Constants.Swerve.Mod0.angleOffset,
                Constants.Swerve.Mod0.angleInverted,
                Constants.Swerve.Mod0.driveInverted),

            new SwerveModule(1,
                Constants.Swerve.Mod1.driveMotorID,
                Constants.Swerve.Mod1.angleMotorID,
                Constants.Swerve.Mod1.angleEncoderID,
                Constants.Swerve.Mod1.angleOffset,
                Constants.Swerve.Mod1.angleInverted,
                Constants.Swerve.Mod1.driveInverted),

            new SwerveModule(2,
                Constants.Swerve.Mod2.driveMotorID,
                Constants.Swerve.Mod2.angleMotorID,
                Constants.Swerve.Mod2.angleEncoderID,
                Constants.Swerve.Mod2.angleOffset,
                Constants.Swerve.Mod2.angleInverted,
                Constants.Swerve.Mod2.driveInverted),

            new SwerveModule(3,
                Constants.Swerve.Mod3.driveMotorID,
                Constants.Swerve.Mod3.angleMotorID,
                Constants.Swerve.Mod3.angleEncoderID,
                Constants.Swerve.Mod3.angleOffset,
                Constants.Swerve.Mod3.angleInverted,
                Constants.Swerve.Mod3.driveInverted),

        };

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroAngle(), getModulePositions());
        fieldRelative = true;
    }

    /**
     * Resets the swerve module encoders to the position of the absolute encoders.
     */
    public void resetEncoders() {
        for (SwerveModule mod : swerveModules) {
            swerveModules[mod.moduleNumber].resetToAbsolute();
        }
    }

    /**
     * Drives the swerve drive based on translation and rotation inputs.
     *
     * @param translation The translation vector representing movement in x and y directions.
     * @param rotation    The rotation value for the swerve drive.
     */
    public void drive(Translation2d translation, double rotation) {
        ChassisSpeeds chassisSpeeds = fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getGyroAngle()): new ChassisSpeeds(translation.getX(), translation.getY(), rotation);

        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        setModuleStates(swerveModuleStates);

    }

    /**
     * Sets the desired states for swerve modules.
     *
     * @param desiredStates The array of desired swerve module states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : swerveModules) {
            mod.setDesiredState(desiredStates[mod.moduleNumber]);
        }

    }

     /**
     * Gets the current pose (position and orientation) of the robot.
     *
     * @return The current robot pose.
     */
    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();

    }

    /**
    * Converts a pose from robot space to a pose in the SwerveOdometry.getPoseMeters() space.
    * <ul>
    *     <li>(0, 0) in robot space is the robot itself</li>
    *     <li>(1, 0) is 1 meter in from of the robot</li>
    *     <li>(0, 1) is 1 meter to the right of the robot</li>
    * </ul>
    * @param robotSpacePose A pose in robot space
    * @return The specified pose in swerve space
    */
    public Pose2d poseFromRobotSpace(Pose2d robotSpacePose) {
        Pose2d robot_pose = this.getPose();

        return robotSpacePose.relativeTo(robot_pose);
        
    }

     /**
     * Sets the pose (position and orientation) of the robot.
     *
     * @param pose The desired pose for the robot.
     */
    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroAngle(), getModulePositions(), pose);

    }

     /**
     * Gets the current states of swerve modules.
     *
     * @return The array of current swerve module states.
     */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : swerveModules) {
            states[mod.moduleNumber] = mod.getState();
        }

        return states;

    }

    /**
     * Gets the positions of swerve modules.
     *
     * @return The array of swerve module positions.
     */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : swerveModules) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        
        return positions;

    }

    /**
     * Resets the gyroscope's yaw to zero.
     */
    public void zeroGyro() {
        pidgey.setYaw(0);

    }
    
    /**
     * Gets the current gyro angle (yaw) of the robot.
     *
     * @return The current gyro angle.
     */
    public Rotation2d getGyroAngle() {
        double yaw = pidgey.getYaw().refresh().getValue();
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - yaw)
                : Rotation2d.fromDegrees(yaw);

    }

    @Override

    public void periodic() {
        swerveOdometry.update(getGyroAngle(), getModulePositions());

        for (SwerveModule mod : swerveModules) {
            SmartDashboard.putString("Mod " + mod.moduleNumber, mod.getState().toString());
            

        }

    }

}