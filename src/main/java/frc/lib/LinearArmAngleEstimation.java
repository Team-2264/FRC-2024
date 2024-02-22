package frc.lib;

public class LinearArmAngleEstimation extends AngleEstimation {
    public LinearTrajectoryParameters trajectoryParameters;

    public double goalDistance = 0;

    public LinearArmAngleEstimation(LinearTrajectoryParameters parameters, double initialGuess, double goalDistance) {
        this.trajectoryParameters = parameters;
        this.goalDistance = goalDistance;
        this.estimate = initialGuess;

        recalculateAccuracy();
    }

    /**
     * Normalizes the estimate so that it's in the range [0, 2 * Math.PI)
     */
    public void normalize() {
        double max_value = trajectoryParameters.launchAngleOffset + Math.PI / 2;
        estimate %= max_value;
        if(estimate < 0) {
            // Generally in mathematics, `a mod b` is in the range [0, b).
            // For some reason, this is not true in many programming languages.
            // Instead, if a < 0, `a % b` is negative.

            // Therefore, we need to do the following to have reasonable behavior.
            estimate += max_value;
        }
    }

    /**
     * Improves the estimate using Newton's Method
     */
    public void doNewtonIteration() {
        this.estimate = estimate - inaccuracy/dHeight();

        recalculateAccuracy();
        normalize();
    }

    public void recalculateAccuracy() {
        this.inaccuracy = projectedHeight() - trajectoryParameters.goalHeight;
    }
    
    /// Gets the height of a trajectory for a given alpha and x
    private double projectedHeight() {
        // See https://www.desmos.com/calculator/umxrqj6n0w

        final double alpha = estimate;
        final double x = goalDistance;

        final double L = trajectoryParameters.armLength;
        final double beta = trajectoryParameters.launchAngleOffset;

        return (x-L*Math.cos(alpha))*Math.tan(alpha-beta)+L*Math.sin(alpha);
    }

    /// Gets the partial derivative with respect to alpha of projectedHeight
    public double dHeight() {
        final double alpha = estimate;
        final double x = goalDistance;

        final double L = trajectoryParameters.armLength;
        final double beta = trajectoryParameters.launchAngleOffset;

        final double cos_alpha_minus_beta = Math.cos(alpha - beta);

        return 1.0/(cos_alpha_minus_beta * cos_alpha_minus_beta)*(L*Math.cos(alpha-2.0*beta)-L*Math.cos(alpha)+2*x)/2.0;
    }
}
