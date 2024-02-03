package frc.lib.motors;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class TalonFx {
    com.ctre.phoenix6.hardware.TalonFX inner;

    public TalonFx(TalonFxConfiguration configuration) {
        this(configuration.canId);

        applyConfiguration(configuration);
    }

    public TalonFx(int canId) {
        inner = new com.ctre.phoenix6.hardware.TalonFX(canId);
    }

    public TalonFx withConfiguration(TalonFxConfiguration configuration) {
        this.applyConfiguration(configuration);
        return this;
    }

    public void applyConfiguration(TalonFxConfiguration c) {
        var raw_config = new com.ctre.phoenix6.configs.TalonFXConfiguration();
        raw_config.CurrentLimits = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(c.continuousCurrentLimit)
            .withSupplyCurrentThreshold(c.peakCurrentLimit)
            .withSupplyTimeThreshold(c.peakCurrentDuration)
            .withSupplyCurrentLimitEnable(c.enableCurrentLimit);

        raw_config.Slot0.kP = c.kP;
        raw_config.Slot0.kI = c.kI;
        raw_config.Slot0.kD = c.kD;
        raw_config.Slot0.kV = c.kF;

        raw_config.OpenLoopRamps = new OpenLoopRampsConfigs().withDutyCycleOpenLoopRampPeriod(c.openLoopRamp);
        raw_config.ClosedLoopRamps = new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(c.closedLoopRamp);

        if(c.neutralModeBrake) {
            raw_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        } else {
            raw_config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        }

        inner.getConfigurator().apply(raw_config);
    }

    public double getVelocity() {
        return inner.getVelocity().refresh().getValue();
    }

    public void rotateTo(double positionRevs) {
        inner.setControl(new PositionDutyCycle(positionRevs));
    }

    /**
     * Rotates the motor at a specific speed
     * 
     * @param velocity desired velocity of the motor in rotations per second
     */
    public void rotateWithVelocity(double velocity) {
        inner.setControl(new VelocityDutyCycle(velocity));
    }

    /**
     * Rotates the motor at a specific speed
     * 
     * @param velocity desired velocity of the motor in rotations per second
     */
    public void rotateWithVelocity(double velocity, double feedForward) {
        inner.setControl(new VelocityDutyCycle(velocity).withFeedForward(feedForward));
    }

    public double getPosition() {
        return inner.getPosition().refresh().getValue();
    }

    public void setSensorPosition(double positionRevs) {
        inner.setPosition(positionRevs);
    }
}
