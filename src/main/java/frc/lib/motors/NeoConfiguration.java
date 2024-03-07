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

    /**
     * Creates a Neo Motor configuration using default values.
     */
    public NeoConfiguration(int canId) {
        this.canId = canId;
    }

    /**
     * Sets the Neo to brake when it isn't being controlled.
     */
    public NeoConfiguration withBrakeMode(boolean brake_mode) {
        this.breakMode = brake_mode;
        return this;
    }

    /**
     * Sets the voltage of the Neo to the voltage of a different Neo.
     */
    public NeoConfiguration followMotor(int canId) {
        this.followMotor = OptionalInt.of(canId);
        return this;
    }

    /**
     * When following another motor, move in the opposite direction.
     */
    public NeoConfiguration withFollowInvert(boolean invert) {
        this.followInvert = invert;
        return this;
    }

    /**
     * Sets the PID kP value.
     */
    public NeoConfiguration withKP(double kP) {
        this.kP = kP;
        return this;
    }

    /**
     * Sets the PID kI value.
     */
    public NeoConfiguration withKI(double kI) {
        this.kI = kI;
        return this;
    }

    /**
     * Sets the PID kD value.
     */
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
