package frc.lib.motors;

public class TalonFxConfiguration {
    public int canId;
    
    public double kP = 0.1;
    public double kI = 0.0;
    public double kD = 0.0;
    public double kF = 0.0;

    public boolean enableCurrentLimit = true;
    public int continuousCurrentLimit = 25;
    public int peakCurrentLimit = 40;
    public double peakCurrentDuration = 0.1;

    public boolean neutralModeBrake = false;

    public double openLoopRamp = 0.5;
    public double closedLoopRamp = 0.5;

    public TalonFxConfiguration(int canId) {
        this.canId = canId;
    }

    public TalonFxConfiguration withPID(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        return this;
    }

    public TalonFxConfiguration withContinuousCurrentLimit(int continuousCurrentLimit) {
        this.continuousCurrentLimit = continuousCurrentLimit;
        this.enableCurrentLimit = true;
        return this;
    }
    public TalonFxConfiguration withPeakCurrentLimit(int peakCurrentLimit) {
        this.peakCurrentLimit = peakCurrentLimit;
        this.enableCurrentLimit = true;
        return this;
    }
    public TalonFxConfiguration withPeakCurrentDuration(double peakCurrentDuration) {
        this.peakCurrentDuration = peakCurrentDuration;
        this.enableCurrentLimit = true;
        return this;
    }

    public TalonFxConfiguration withoutCurrentLimit() {
        this.enableCurrentLimit = false;
        return this;
    }

    public TalonFxConfiguration withNeutralModeBrake() {
        this.neutralModeBrake = true;
        return this;
    }

    public TalonFxConfiguration withoutNeutralModeBrake() {
        this.neutralModeBrake = false;
        return this;
    }

    public TalonFxConfiguration withOpenLoopRamp(double openLoopRamp) {
        this.openLoopRamp = openLoopRamp;
        return this;
    }
    public TalonFxConfiguration withClosedLoopRamp(double closedLoopRamp) {
        this.closedLoopRamp = closedLoopRamp;
        return this;
    }
}
