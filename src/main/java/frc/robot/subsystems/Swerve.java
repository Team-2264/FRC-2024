package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

import java.util.List;

import com.ctre.phoenix6.configs.Pigeon2Configuration;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The Swerve class represents a swerve drive subsystem for a robot.
 * This class handles swerve modules, odometry, and gyro integration.
 */
public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    private final Field2d field;
    public Pigeon2 pidgey;

    public SwerveModule[] swerveModules;

    private boolean fieldRelative = true;

    /**
     * Constructs a Swerve subsystem instance.
     */
    public Swerve() {
        // Init pigeon
        pidgey = new Pigeon2(Constants.Swerve.pigeonID);
        pidgey.getConfigurator().apply(new Pigeon2Configuration());
        zeroGyro();

        // Init Modules
        swerveModules = new SwerveModule[] {
            new SwerveModule(0,
                Constants.Swerve.Mod0.driveMotorID,
                Constants.Swerve.Mod0.angleMotorID,
                Constants.Swerve.Mod0.angleEncoderID,
                Constants.Swerve.Mod0.angleOffset),

            new SwerveModule(1,
                Constants.Swerve.Mod1.driveMotorID,
                Constants.Swerve.Mod1.angleMotorID,
                Constants.Swerve.Mod1.angleEncoderID,
                Constants.Swerve.Mod1.angleOffset),

            new SwerveModule(2,
                Constants.Swerve.Mod2.driveMotorID,
                Constants.Swerve.Mod2.angleMotorID,
                Constants.Swerve.Mod2.angleEncoderID,
                Constants.Swerve.Mod2.angleOffset),

            new SwerveModule(3,
                Constants.Swerve.Mod3.driveMotorID,
                Constants.Swerve.Mod3.angleMotorID,
                Constants.Swerve.Mod3.angleEncoderID,
                Constants.Swerve.Mod3.angleOffset),

        };

        // Init Odometry
        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroAngle(), getModulePositions());

        // Configure AutoBuilder
        AutoBuilder.configureHolonomic(
            this::getPose, 
            this::setPose, 
            this::getChassisSpeeds, 
            this::drive, 
            Constants.Swerve.pathFollowerConfig,
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this
        );

        // Init Field
        field = new Field2d();

        // Logging callback for current robot pose
        PathPlannerLogging.setLogCurrentPoseCallback((Pose2d pose) -> {
            field.setRobotPose(pose);

        });

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback((Pose2d pose) -> {
            field.getObject("target pose").setPose(pose);

        });

        // Logging callback for the active path, this is sent as a list of poses
        PathPlannerLogging.setLogActivePathCallback((List<Pose2d> poses) -> {
            field.getObject("path").setPoses(poses);

        });
        
    }
    
    /**
     * Drives the swerve drive based on field-relative {@link ChassisSpeeds}.
     */
    public void drive(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);

        setModuleStates(swerveModuleStates);
    }
    /**
     * Drives the swerve drive based on translation and rotation inputs.
     *
     * @param translation The translation vector representing movement in x and y directions.
     * @param rotation    The rotation value for the swerve drive.
     */
    public void drive(Translation2d translation, double rotation) {
        Translation2d limitedTranslation = translation;

        if(translation.getNorm() > Constants.Swerve.maxSpeed) {
            limitedTranslation = translation.div(translation.getNorm() / Constants.Swerve.maxSpeed);
        }

        ChassisSpeeds chassisSpeeds = fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(limitedTranslation.getX(), limitedTranslation.getY(), rotation, getGyroAngle()): new ChassisSpeeds(limitedTranslation.getX(), limitedTranslation.getY(), rotation);

        drive(chassisSpeeds);
    }


    /**
     * Sets the desired states for swerve modules.
     *
     * @param desiredStates The array of desired swerve module states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
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
     * Gets the field-relative {@link ChassisSpeeds} for the robot.
     */
    public ChassisSpeeds getChassisSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
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

    /**
     * Generates a command that will follow a specified path.
     * 
     * @param pathName
     * @return
     */
    public Command followPathCommand(PathPlannerPath path) {
        return new FollowPathHolonomic(
                path,
                this::getPose, // Robot pose supplier
                this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                Constants.Swerve.pathFollowerConfig, // PathFollowerConfig
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;

                },
                this // Reference to this subsystem to set requirements
        );

    }

    @Override
    public void periodic() {
        swerveOdometry.update(getGyroAngle(), getModulePositions());

        for (SwerveModule mod : swerveModules) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber, mod.getState().angle.getDegrees());

        }

    }

}
