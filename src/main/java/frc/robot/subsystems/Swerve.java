package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.lib.Math2264;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.configs.Pigeon2Configuration;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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
    private final Field2d field;
    public Pigeon2 pidgey;

    public SwerveModule[] swerveModules;

    public boolean turboModeStatus = false;

    private boolean fieldRelative = true;

    private PIDController rotationLockController = new PIDController(Constants.Swerve.rotationLockKP, Constants.Swerve.rotationLockKI, Constants.Swerve.rotationLockKD);
    private Optional<Translation2d> lockedOnto = Optional.empty();

    public final SwerveDrivePoseEstimator poseEstimator;

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

        poseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getGyroAngle(), getModulePositions(), Constants.Swerve.initialPose);

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
    
    public void lockOnto(Translation2d translation2d) {
        rotationLockController.reset();
        
        lockedOnto = Optional.of(translation2d);
    }

    public void unlockRotation() {
        lockedOnto = Optional.empty();
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
        if(lockedOnto.isPresent()) {
            Pose2d robot_pose = getPose();

            Translation2d look_vector = lockedOnto.get().minus(robot_pose.getTranslation()); // Vector from robot to target
            Rotation2d look_rotation = new Rotation2d(look_vector.getX(), look_vector.getY()).rotateBy(new Rotation2d(Math.PI)); // Rotation of vector

            double targetRotationOffest = MathUtil.angleModulus(look_rotation.minus(robot_pose.getRotation()).getRadians());

            double targetRotation = robot_pose.getRotation().getRadians() + targetRotationOffest;
            if (Math.abs(targetRotationOffest) < Math.toRadians(2)) {
                rotation = 0;
            } else {
                rotation = rotationLockController.calculate(robot_pose.getRotation().getRadians(), targetRotation) * 3;
            }
        }

        rotation = Math2264.limitMagnitude(rotation, Constants.Swerve.maxAngularVelocity);
        Translation2d limitedTranslation = Math2264.limitMagnitude(translation, maximumSpeed());

        ChassisSpeeds chassisSpeeds = fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(limitedTranslation.getX(), limitedTranslation.getY(), rotation, getGyroAngle()): new ChassisSpeeds(limitedTranslation.getX(), limitedTranslation.getY(), rotation);

        drive(chassisSpeeds);
    }

    /**
     * Get maximum speed based on turbo mode status
     */
    public double maximumSpeed(){
        if(turboModeStatus == true){
            return Constants.Swerve.turboMaxSpeed;
        }else{
            return Constants.Swerve.maxSpeed;
        }
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
        return poseEstimator.getEstimatedPosition();

    }

     /**
     * Sets the pose (position and orientation) of the robot.
     *
     * @param pose The desired pose for the robot.
     */
    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(getGyroAngle(), getModulePositions(), pose);

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

    /**
     * Turns on/off turbo mode
     */
    public void toggleTurboMode(boolean condition){
        turboModeStatus = condition;
    }
    
    /** 
     * Returns if turbo mode is on
     */ 
    public boolean getTurboStatus(){
        return turboModeStatus;
    }

    /**
     * Adds a vision measurement to the pose estimator. This will correct the robot's pose.
     * 
     * @param pose The estimated robot pose.
     */
    public void addVisionMeasurement(EstimatedRobotPose pose) {
        if(pose.targetsUsed.size() == 1) {
            return;
        }

        Pose2d measuredPose = pose.estimatedPose.toPose2d();


        double totalDistance = 0;
        for(PhotonTrackedTarget target: pose.targetsUsed) {
            totalDistance += target.getBestCameraToTarget().getTranslation().getNorm();
        }
        double averageDistance = totalDistance / pose.targetsUsed.size();

        double devMultiplier = (1.0 /3.0) * averageDistance;
        Matrix<N3, N1> standardDevs = Constants.Vision.visionStandardDevs.times(devMultiplier);

        poseEstimator.addVisionMeasurement(measuredPose, pose.timestampSeconds, standardDevs);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("TURBO", turboModeStatus);

        field.setRobotPose(getPose());

        SmartDashboard.putData(field);

        poseEstimator.update(getGyroAngle(), getModulePositions());

        for (SwerveModule mod : swerveModules) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber, mod.getState().angle.getDegrees());

        }

    }

}
