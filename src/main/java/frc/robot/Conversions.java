package frc.robot;

/**
 * A utility class for performing unit conversions.
 */
public class Conversions {
    /**
     * Converts Falcon motor counts to degrees of rotation of a mechanism.
     *
     * @param counts    Falcon motor counts
     * @param gearRatio Gear ratio between Falcon and mechanism
     * @return Degrees of rotation of the mechanism
     */
    public static double falconToDegrees(double counts, double gearRatio) {
        return counts * (360.0 / (gearRatio * 2048.0));

    }

   /**
   * Converts degrees of rotation of a mechanism to Falcon motor counts.
   *
   * @param degrees   Degrees of rotation of the mechanism
   * @param gearRatio Gear ratio between Falcon and mechanism
   * @return Falcon motor counts
   */
    public static double degreesToFalcon(double degrees, double gearRatio) {
        double ticks = degrees / (360.0 / (gearRatio * 2048.0));
        return ticks;

    }

    /**
     * Converts Falcon motor velocity counts to RPM of a mechanism.
     *
     * @param velocityCounts Falcon motor velocity counts
     * @param gearRatio      Gear ratio between Falcon and mechanism (set to 1 for Falcon RPM)
     * @return RPM of the mechanism
     */
    public static double falconToRPM(double velocityCounts, double gearRatio) {
        double motorRPM = velocityCounts * (600.0 / 2048.0);
        double mechRPM = motorRPM / gearRatio;
        return mechRPM;

    }

    /**
     * Converts RPM of a mechanism to Falcon motor velocity counts.
     *
     * @param RPM       RPM of the mechanism
     * @param gearRatio Gear ratio between Falcon and mechanism (set to 1 for Falcon RPM)
     * @return Falcon motor velocity counts
     */
    public static double RPMToFalcon(double RPM, double gearRatio) {
        double motorRPM = RPM * gearRatio;
        double sensorCounts = motorRPM * (2048.0 / 600.0);
        return sensorCounts;

    }

    /**
     * Converts Falcon motor velocity counts to meters per second (MPS) of a mechanism.
     *
     * @param velocityCounts Falcon motor velocity counts
     * @param circumference  Circumference of the wheel
     * @param gearRatio      Gear ratio between Falcon and mechanism (set to 1 for Falcon RPM)
     * @return MPS of the mechanism
     */
    public static double falconToMPS(double velocitycounts, double circumference, double gearRatio) {
        double wheelRPM = falconToRPM(velocitycounts, gearRatio);
        double wheelMPS = (wheelRPM * circumference) / 60;
        return wheelMPS;

    }

    /**
     * Converts Falcon motor velocity counts to meters per second (MPS) of a mechanism.
     *
     * @param velocityCounts Falcon motor velocity counts
     * @param circumference  Circumference of the wheel
     * @param gearRatio      Gear ratio between Falcon and mechanism (set to 1 for Falcon RPM)
     * @return MPS of the mechanism
     */
    public static double MPSToFalcon(double velocity, double circumference, double gearRatio) {
        double wheelRPM = ((velocity * 60) / circumference);
        double wheelVelocity = RPMToFalcon(wheelRPM, gearRatio);
        return wheelVelocity;
        
    }

    /**
     * Converts Falcon motor position counts to meters of a mechanism.
     *
     * @param positionCounts Falcon motor position counts
     * @param circumference  Circumference of the wheel
     * @param gearRatio      Gear ratio between Falcon and mechanism (set to 1 for Falcon RPM)
     * @return Meters of the mechanism
     */
    public static double falconToMeters(double positionCounts, double circumference, double gearRatio) {
        return positionCounts * (circumference / (gearRatio * 2048.0));

    }



    // NEW CONVERSIONS
    public static double revsToMeters(double revs, double circumference, double gearRatio) {
        return (revs * circumference) / gearRatio;

    }

    public static double metersToRevs(double meters, double circumference, double gearRatio) {
        return (meters / circumference) * gearRatio;

    }

    public static double revsToDegrees(double revs, double gearRatio) {
        return (revs * 360) / gearRatio;

    }

    public static double degreesToRevs(double degrees, double gearRatio) {
        return (degrees / 360) * gearRatio;

    }


}
