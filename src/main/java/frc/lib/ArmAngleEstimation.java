package frc.lib;

public class ArmAngleEstimation {
    public TrajectoryParameters trajectoryParameters;

    public double estimate = 0;
    public double inaccuracy = 0;

    public double goalDistance = 0;

    public ArmAngleEstimation(TrajectoryParameters parameters, double initialGuess, double goalDistance) {
        this.trajectoryParameters = parameters;
        this.goalDistance = goalDistance;
        this.estimate = initialGuess;

        recalculateAccuracy();
    }

    /**
     * Normalizes the estimate so that it's in the range [0, 2 * Math.PI)
     */
    public void normalize() {
        double max_value = Math.PI / 2;
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
        final double v_0 = trajectoryParameters.launchVelocity;
        final double g = trajectoryParameters.gravity;

        final double sin_alpha = Math.sin(alpha);
        final double cos_alpha = Math.cos(alpha);

        final double cos_beta_minus_alpha = Math.cos(beta - alpha);
        final double sin_beta_minus_alpha = Math.sin(beta - alpha);

        return ((2*L*v_0*v_0*cos_alpha-2*v_0*v_0*x)*cos_beta_minus_alpha*
        sin_beta_minus_alpha+2*L*v_0*v_0*sin_alpha*cos_beta_minus_alpha*
        cos_beta_minus_alpha-L*L*g*cos_alpha*cos_alpha+2*L*g*x*cos_alpha-g*
        x*x)/(2*v_0*v_0*cos_beta_minus_alpha*cos_beta_minus_alpha);
    }

    /// Gets the partial derivative with respect to alpha of projectedHeight
    public double dHeight() {
        final double alpha = estimate;
        final double x = goalDistance;

        final double L = trajectoryParameters.armLength;
        final double beta = trajectoryParameters.launchAngleOffset;
        final double v_0 = trajectoryParameters.launchVelocity;
        final double g = trajectoryParameters.gravity;

        final double sin_alpha = Math.sin(alpha);
        final double cos_alpha = Math.cos(alpha);

        final double cos_beta_minus_alpha = Math.cos(beta - alpha);
        final double sin_beta_minus_alpha = Math.sin(beta - alpha);

        return -(((L*v_0*v_0*cos_alpha-v_0*v_0*x)*cos_beta_minus_alpha*sin_beta_minus_alpha*sin_beta_minus_alpha*sin_beta_minus_alpha+(L*v_0*v_0*sin_alpha*cos_beta_minus_alpha*cos_beta_minus_alpha-L*L*g*cos_alpha*cos_alpha+2*L*g*x*cos_alpha-g*x*x)*sin_beta_minus_alpha-v_0*v_0*x*cos_beta_minus_alpha*cos_beta_minus_alpha*cos_beta_minus_alpha+(L*g*x-L*L*g*cos_alpha)*sin_alpha*cos_beta_minus_alpha)/(v_0*v_0*cos_beta_minus_alpha*cos_beta_minus_alpha*cos_beta_minus_alpha));
    }
}
