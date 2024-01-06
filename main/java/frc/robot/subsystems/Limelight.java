package frc.robot.subsystems;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A subsystem for the Limelight.
 */
public class Limelight extends SubsystemBase {
    private NetworkTable table;

    public Limelight() {
       table = NetworkTableInstance.getDefault().getTable("limelight");

    }

    public boolean hasTarget() {
        return table.getEntry("tv").getDouble(0) == 1;
    }

    public static enum ApriltagPose {
        CPTS("camerapose_targetspace"),
        TPCS("targetpose_cameraspace"),
        TPRS("targetpose_robotspace"),
        BPTS("botpose_targetspace"),
        CPRS("camerapose_robotspace");
        
        public final String label;

        private ApriltagPose(String label) {
            this.label = label;
        }

    }

    public double[] getPose(ApriltagPose key) {
        return table.getEntry(key.label).getDoubleArray(new double[6]);

    }

    public void periodic() {
        SmartDashboard.putBoolean("Tag", hasTarget());

        SmartDashboard.putString("CPTS", Arrays.toString(getPose(ApriltagPose.CPTS))); // VAILD IN Target Space
        SmartDashboard.putString("TPRS", targetPoseRelativeToRobot(getPose(ApriltagPose.TPRS)).toString());
    }

    /// Gets the position of the tag relative to the robot in meters. Where:
    /// - The robot is at 0, 0, 0 @ 0 degrees 
    /// - X+ is forwards
    /// - Y+ is to the left
    /// - Positive rotation is rotating to the left
    public static Pose2d targetPoseRelativeToRobot(double[] robot_space) {
        return new Pose2d(new Translation2d(robot_space[2], -robot_space[0]), new Rotation2d(-Math.toRadians(robot_space[4])));
    }
}
