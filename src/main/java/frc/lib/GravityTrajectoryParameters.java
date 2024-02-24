package frc.lib;

public class GravityTrajectoryParameters {
    /** Length of arm (meters) */
    public double armLength = 0.5;

    /** Goal height relative to arm pivot point (meters) */
    public double goalHeight = 2;

    /**
     * The launch angle clockwise offset (radians).
     * <ul>
     *   <li>an offset of 0 radians will shoot parallel with the arm.</li>
     *   <li>an offset of pi/4 radians with an arm angle of pi/2 will shoot in the positive x direction.</li>
     * <ul>
    */
    public double launchAngleOffset = Math.PI / 4;

    /** Initial velocity of arm in meters per second */
    public double launchVelocity = 30.0;

    public double gravity = 9.8;

    /**
     * Returns the estimated angle that the arm must be at to hit a target `goalDistance` units away
     * @param newtonIterations The number of newton iterations to be performed. The higher the number,
     *     the more accurate the estimation is, but it will require significantly more computation time.
     * @param initialGuess The initial arm angle in radians used for newton iterations.
     * @param goalDistance The horizontal distance to the goal in meters.
     */
    public GravityArmAngleEstimation getEstimate(double goalDistance, double initialGuess, int newtonIterations) {
        GravityArmAngleEstimation estimate = new GravityArmAngleEstimation(this, initialGuess, goalDistance);

        for(int i = 0; i < newtonIterations; i++) {
            estimate.doNewtonIteration();
        }

        return estimate;
    }

    /**
     * Returns the estimated angle that the arm must be at to hit a target `goalDistance` units away
     * @param newtonIterations The number of newton iterations to be performed. The higher the number,
     *     the more accurate the estimation is, but it will require significantly more computation time.
     * @param goalDistance The horizontal distance to the goal in meters
     */
    public GravityArmAngleEstimation getEstimate(double goalDistance, int newtonIterations) {
        return getEstimate(goalDistance, 3 * Math.PI / 4, newtonIterations);
    }

    /**
     * Sets the length of arm
     * @param armLength length of the arm in meters
     */
    public GravityTrajectoryParameters withArmLength(double armLength) {
        this.armLength = armLength;
        return this;
    }
    
   
    /** 
     * Sets the height of the goal
     * @param goalHeight height of the goal relative to the arm's pivot point in meters
     */
    public GravityTrajectoryParameters withGoalHeight(double goalHeight) {
        this.goalHeight = goalHeight;
        return this;
    }

    public GravityTrajectoryParameters withLaunchVelocity(double launchVelocity) {
        this.launchVelocity = launchVelocity;
        return this;
    }

    /**
     * Sets the launch angle offset.
     * <ul>
     *   <li>an offset of 0 radians will shoot parallel with the arm.</li>
     *   <li>an offset of pi/4 radians with an arm angle of pi/2 will shoot in the positive x direction.</li>
     * <ul>
     * @param launchAngleOffset Clockwise launch angle offset (radians)
    */
    public GravityTrajectoryParameters withLaunchAngleOffset(double launchAngleOffset) {
        this.launchAngleOffset = launchAngleOffset;
        return this;
    }
}