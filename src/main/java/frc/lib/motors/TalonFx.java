package frc.lib.motors;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.controls.PositionDutyCycle;

public class TalonFx {
    com.ctre.phoenix6.hardware.TalonFX inner;

    public TalonFx(TalonFxConfiguration configuration) {
        inner = new com.ctre.phoenix6.hardware.TalonFX(configuration.canId);

        applyConfiguration(configuration);
    }

    public void applyConfiguration(TalonFxConfiguration c) {
        var raw_config = new com.ctre.phoenix6.configs.TalonFXConfiguration();
        raw_config.CurrentLimits = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(c.continuousCurrentLimit)
            .withSupplyCurrentThreshold(c.peakCurrentLimit)
            .withSupplyTimeThreshold(c.continuousCurrentLimit)
            .withSupplyCurrentLimitEnable(c.enableCurrentLimit);

        raw_config.Slot0.kP = c.kP;
        raw_config.Slot0.kI = c.kI;
        raw_config.Slot0.kD = c.kD;
        raw_config.Slot0.kV = c.kF;

        raw_config.OpenLoopRamps = new OpenLoopRampsConfigs().withDutyCycleOpenLoopRampPeriod(c.openLoopRamp);
        raw_config.ClosedLoopRamps = new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(c.closedLoopRamp);

        inner.getConfigurator().apply(raw_config);
    }

    public double getVelocity() {
        return inner.getVelocity().refresh().getValue();
    }

    public void rotateTo(double positionRevs) {
        inner.setControl(new PositionDutyCycle(positionRevs));
    }

    public double getPosition() {
        return inner.getPosition().refresh().getValue();
    }

    public void setSensorPosition(double positionRevs) {
        inner.setPosition(positionRevs);
    }
}
