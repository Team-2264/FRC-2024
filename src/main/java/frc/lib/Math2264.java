package frc.lib;

import edu.wpi.first.math.geometry.Translation2d;

public abstract class Math2264 {
    /**
     * Forces the input value to be between two numbers.
     * @param min the minimum value
     * @param val the value to clamp
     * @param max the maximum value
     */
    public static double clamp(double min, double val, double max) {
        return Math.max(min, Math.min(val, max));
    }

    /**
     * Clamps the value between `-magnitude` and `magnitude`.
     */
    public static double limitMagnitude(double val, double magnitude) {
        return clamp(-magnitude, val, magnitude);
    }

    /**
     * Clamps the `norm` of the vector `val` between `0` and `magnitude`.
     */
    public static Translation2d limitMagnitude(Translation2d val, double magnitude) {
        double norm = val.getNorm();
        if(norm < magnitude) {
            return val;
        } else if(magnitude == 0) {
            return new Translation2d();
        }

        return val.times(magnitude/norm);
    }
}
