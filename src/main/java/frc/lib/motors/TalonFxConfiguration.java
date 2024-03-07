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

    public double openLoopRamp = 0;
    public double closedLoopRamp = 0;

    /**
     * Creates a Talon motor configuration with a specific CAN ID
     */
    public TalonFxConfiguration(int canId) {
        this.canId = canId;
    }

    public TalonFxConfiguration() {
        this.canId = 0;
    }

    /**
     * Sets the Talon's PID values
     */
    public TalonFxConfiguration withPID(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        return this;
    }

    /**
     * Sets the current limit for the motor. This limit can be exceeded for `peakCurrentDuration` seconds.
     */
    public TalonFxConfiguration withContinuousCurrentLimit(int continuousCurrentLimit) {
        this.continuousCurrentLimit = continuousCurrentLimit;
        this.enableCurrentLimit = true;
        return this;
    }

    /**
     * Sets the peak current limit for the motor. This limit cannot be exceeded.
     */
    public TalonFxConfiguration withPeakCurrentLimit(int peakCurrentLimit) {
        this.peakCurrentLimit = peakCurrentLimit;
        this.enableCurrentLimit = true;
        return this;
    }

    /**
     * How long continuousCurrentLimit can be exceeded in seconds.
     */
    public TalonFxConfiguration withPeakCurrentDuration(double peakCurrentDuration) {
        this.peakCurrentDuration = peakCurrentDuration;
        this.enableCurrentLimit = true;
        return this;
    }

    /** 
     * Disables current limiting.
     */
    public TalonFxConfiguration withoutCurrentLimit() {
        this.enableCurrentLimit = false;
        return this;
    }

    /**
     * Sets the motor to brake mode when it's not being controlled
     */
    public TalonFxConfiguration withNeutralModeBrake() {
        this.neutralModeBrake = true;
        return this;
    }

    /**
     * Sets the motor to coast mode when it's not being controlled
     */
    public TalonFxConfiguration withoutNeutralModeBrake() {
        this.neutralModeBrake = false;
        return this;
    }

    /**
     * Time to ramp open loop motor controller from 0% to 100% output.
     */
    public TalonFxConfiguration withOpenLoopRamp(double openLoopRamp) {
        this.openLoopRamp = openLoopRamp;
        return this;
    }

    /**
     * Time to ramp closed loop motor controller from 0% to 100% output.
     */
    public TalonFxConfiguration withClosedLoopRamp(double closedLoopRamp) {
        this.closedLoopRamp = closedLoopRamp;
        return this;
    }
}
