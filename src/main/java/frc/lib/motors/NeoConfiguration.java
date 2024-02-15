package frc.lib.motors;

import java.util.OptionalInt;

public class NeoConfiguration {
    public int canId;
    
    public double kP = 0.5;
    public double kI = 0.001;
    public double kD = 0;

    public double outputRangeLower = -0.05;
    public double outputRangeUpper = 0.05;

    public boolean breakMode = false;

    public OptionalInt followMotor = OptionalInt.empty();
    public boolean followInvert = false;

    public NeoConfiguration(int canId) {
        this.canId = canId;
    }

    public NeoConfiguration withBrakeMode(boolean brake_mode) {
        this.breakMode = brake_mode;
        return this;
    }

    public NeoConfiguration followMotor(int canId) {
        this.followMotor = OptionalInt.of(canId);
        return this;
    }

    public NeoConfiguration withFollowInvert(boolean invert) {
        this.followInvert = invert;
        return this;
    }

    public NeoConfiguration withKP(double kP) {
        this.kP = kP;
        return this;
    }
    public NeoConfiguration withKI(double kI) {
        this.kI = kI;
        return this;
    }
    public NeoConfiguration withKD(double kD) {
        this.kD = kD;
        return this;
    }

    public NeoConfiguration withOutputRange(double lower, double upper) {
        this.outputRangeUpper = upper;
        this.outputRangeLower = lower;
        return this;
    }
}
