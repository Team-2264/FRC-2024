package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * A utility class for optimizing the behavior of CTRE-driven swerve drive modules.
 */
public class SwerveOptimizer {

    /**
     * Minimize the change in heading the desired swerve module state would require
     * by potentially reversing the direction the wheel spins. Customized from WPILib's version to
     * include placing in appropriate scope for CTRE onboard control.
     *
     * @param desiredState The desired state.
     * @param currentAngle The current module angle.
     * @return The optimized swerve module state.
     */
    public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
        double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
        double targetSpeed = desiredState.speedMetersPerSecond;
        double delta = targetAngle - currentAngle.getDegrees();
        if (Math.abs(delta) > 90) {
            targetSpeed = -targetSpeed;
            targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
        }
        
        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));

    }

    /**
     * Adjusts the angle to ensure it falls within the appropriate 0 to 360-degree scope,
     * while maintaining smooth transitions and avoiding angle wrapping issues.
     *
     * @param scopeReference The current angle used as a reference.
     * @param newAngle       The target angle.
     * @return The adjusted angle within the 0 to 360-degree scope.
     */
    private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
        double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % 360;
        if (lowerOffset >= 0) {
            lowerBound = scopeReference - lowerOffset;
            upperBound = scopeReference + (360 - lowerOffset);
        } else {
            upperBound = scopeReference - lowerOffset;
            lowerBound = scopeReference - (360 + lowerOffset);
        }
        while (newAngle < lowerBound) {
            newAngle += 360;
        }
        while (newAngle > upperBound) {
            newAngle -= 360;
        }
        if (newAngle - scopeReference > 180) {
            newAngle -= 360;
        } else if (newAngle - scopeReference < -180) {
            newAngle += 360;
        }

        return newAngle;

    }

}
