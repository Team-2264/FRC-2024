package frc.robot.subsystems;

import java.util.Arrays;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

/**
 * Subsystem for vision processing using photonvision.
 * 
 */
public class Vision extends SubsystemBase {
    private PhotonCamera camera;
    private PhotonPoseEstimator estimator;

    private PhotonPipelineResult latestResult;

    private Field2d field;

    public Vision() {  
        camera = new PhotonCamera(Constants.Vision.cameraName);
        
        estimator = new PhotonPoseEstimator(
            AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(), 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            camera, Constants.Vision.robotToCamera
                
        );
        estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        estimator.setRobotToCameraTransform(Constants.Vision.robotToCamera);

        field = new Field2d();

    }

    /**
     *  Get the latest result from the camera.
     * 
     */
    private void get() {
        latestResult = camera.getLatestResult();

    }

    /**
     * Estimate the pose of the robot.
     * 
     */
    public Optional<EstimatedRobotPose> getEstimatedPose() {
        // Method 1 - pose estimator
        var robot_pose = estimator.update();
        if(robot_pose.isPresent()) {
            return Optional.of(robot_pose.get());
        }

        return Optional.empty();
    }
    
    /**
     * Periodicly update SmartDashboard.
     */
    @Override
    public void periodic() {
        get(); // Query the camera for the latest result

        // Has targets
        SmartDashboard.putBoolean("Vision - Has Targets", latestResult.hasTargets());

        // Tracked targets ids
        int[] targetIds = new int[latestResult.targets.size()];
        for (int i = 0; i < latestResult.targets.size(); i++) {
            targetIds[i] = latestResult.targets.get(i).getFiducialId();
        }
        SmartDashboard.putString("Vision - Tracked Targets", Arrays.toString(targetIds));

        // Estimated pose
        Optional<EstimatedRobotPose> estimated = getEstimatedPose();
        if (estimated.isPresent()) {
            Pose3d meterPose = estimated.get().estimatedPose;
            field.setRobotPose(meterPose.getX(), meterPose.getY(), meterPose.getRotation().toRotation2d());

            Pose3d inchPose = new Pose3d(
                meterPose.getTranslation().getX() * 39.3701,
                meterPose.getTranslation().getY() * 39.3701,
                meterPose.getTranslation().getZ() * 39.3701,
                meterPose.getRotation()
            );
            SmartDashboard.putString("Vision - Estimated Pose (in)", inchPose.toString());

        }
        
        // Field
        SmartDashboard.putData("Vision - Field", field);
        
    }
    
}
