package frc.robot;

/**
 * A utility class for performing unit conversions.
 */
public class Conversions {
    /** 
     * Converts revolutions to meters.
     * 
     * @param revs The number of revolutions.
     * @param circumference The circumference of the wheel.
     * @param gearRatio The gear ratio of the motor.
     * @return The number of meters.
     * 
     */
    public static double revsToMeters(double revs, double circumference, double gearRatio) {
        return (revs * circumference) / gearRatio;

    }

    /**
     * Converts meters to revolutions.
     * 
     * @param meters The number of meters.
     * @param circumference The circumference of the wheel.
     * @param gearRatio The gear ratio of the motor.
     * @return The number of revolutions.
     * 
     */
    public static double metersToRevs(double meters, double circumference, double gearRatio) {
        return (meters / circumference) * gearRatio;

    }

    /**
     * Converts revolutions to degrees.
     * 
     * @param revs The number of revolutions.
     * @param gearRatio The gear ratio of the motor.
     * @return The number of degrees.
     * 
     */
    public static double revsToDegrees(double revs, double gearRatio) {
        return (revs * 360) / gearRatio;

    }

    /**
     * Converts degrees to revolutions.
     * 
     * @param degrees The number of degrees.
     * @param gearRatio The gear ratio of the motor.
     * @return The number of revolutions.
     * 
     */
    public static double degreesToRevs(double degrees, double gearRatio) {
        return (degrees / 360) * gearRatio;

    }

}
