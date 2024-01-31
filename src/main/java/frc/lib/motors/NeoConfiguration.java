package frc.lib.motors;

public class NeoConfiguration {
    public int canId;
    
    public double kP = 0.5;
    public double kI = 0.001;
    public double kD = 0;

    public double outputRangeLower = -1.0;
    public double outputRangeUpper = 1.0;

    public NeoConfiguration(int canId) {
        this.canId = canId;
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
