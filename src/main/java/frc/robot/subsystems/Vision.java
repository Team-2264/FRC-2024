package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Subsystem for vision processing using photonvision.
 * 
 */
public class Vision extends SubsystemBase {
    private PhotonCamera camera;
    private PhotonPipelineResult latestResult;

    public Vision() {  
        camera = new PhotonCamera("apriltag");

    }

    /**
     *  Get the latest result from the camera.
     * 
     */
    private void get() {
        latestResult = camera.getLatestResult();

    }
    
    /**
     * Periodicly update the dashboard with the latest result.
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
        SmartDashboard.putString("Vision - Tracked Targets", targetIds.toString());

        // Best target data
        if (latestResult.hasTargets()) {
            PhotonTrackedTarget bestTarget = latestResult.getBestTarget();
            
            SmartDashboard.putNumber("Vision - Best Target - ID", bestTarget.getFiducialId());
            Transform3d targetToCamera = bestTarget.getBestCameraToTarget().inverse();
            SmartDashboard.putString("Vision - Best Target - CTT", new Pose3d().plus(targetToCamera).toString());

            Pose3d targetToRobot = new Pose3d().plus(targetToCamera).plus(Constants.Vision.cameraToRobot);

            SmartDashboard.putString("Vision - Best Target - RTT", targetToRobot.toString());


        }
        
    }
    
}
