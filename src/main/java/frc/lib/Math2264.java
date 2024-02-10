package frc.lib;

import edu.wpi.first.math.geometry.Translation2d;

public abstract class Math2264 {
    public static double clamp(double min, double val, double max) {
        return Math.max(min, Math.min(val, max));
    }

    public static double limitMagnitude(double val, double magnitude) {
        return clamp(-magnitude, val, magnitude);
    }

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
