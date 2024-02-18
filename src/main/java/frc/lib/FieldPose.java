package frc.lib;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldPose {
  private static final Translation2d FIELD_SIZE = new Translation2d(Units.inchesToMeters(323.25), Units.inchesToMeters(651.25));

  // Converts between WpiLib Blue and Red field spaces
  private static final Transform3d WPILIB_COLOR_TRANSFORM = new Transform3d(new Translation3d(FIELD_SIZE.getX(), FIELD_SIZE.getY(), 0), new Rotation3d(Math.PI, 0, 0));

  private Pose3d bluePose;

  /**
   * Creates a FieldPose from a WpiLib blue pose
  */
  public static FieldPose fromWpiBlue(Pose3d pose3d) {
    return new FieldPose(pose3d);
  }

  /**
   * Creates a FieldPose from a WpiLib red pose
  */
  public static FieldPose fromWpiRed(Pose3d pose3d) {
    return new FieldPose(pose3d.transformBy(WPILIB_COLOR_TRANSFORM));
  }

  /**
   * Creates a FieldPose from a Pose in the current Alliance's coordinate space.
   */
  public static FieldPose fromNative(Pose3d pose3d) {
    Optional<Alliance> alliance2 = DriverStation.getAlliance();
    if(alliance2.isEmpty()) {
      throw new RuntimeException("Failed to get native pose: Alliance is not set in Driverstation");
    }

    Alliance alliance = alliance2.get();

    if(alliance == Alliance.Red) {
      return fromWpiRed(pose3d);
    } else {
      return fromWpiBlue(pose3d);
    }
  }

  /**
   * Gets the pose as a WPILib Red pose.
   */
  public Pose3d getWpiRed() {
    return bluePose.transformBy(WPILIB_COLOR_TRANSFORM);
  }

  /**
   * Gets the pose as a WPILib Blue pose.
   */
  public Pose3d getWpiBlue() {
    return bluePose;
  }

  /**
   * Gets the Pose as a pose in the current team's coordinate space
   */
  public Pose3d get() {
    Optional<Alliance> alliance2 = DriverStation.getAlliance();
    if(alliance2.isEmpty()) {
      throw new RuntimeException("Failed to get native pose: Alliance is not set in Driverstation");
    }

    Alliance alliance = alliance2.get();

    if(alliance == Alliance.Red) {
      return getWpiRed();
    } else {
      return getWpiBlue();
    }
  }

  /**
   * Creates a FieldPose from a WPILIB Blue pose
   */
  private FieldPose(Pose3d bluePose) {
    this.bluePose = bluePose;
  }
}
