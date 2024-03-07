package frc.lib.motors;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class CanCoder {
  com.ctre.phoenix6.hardware.CANcoder inner;

  /**
   * Creates a CANCoder with some sane default configurations.
   */
  public CanCoder(int id) {
    inner = new CANcoder(id);

    var config = new com.ctre.phoenix6.configs.CANcoderConfiguration();
    config.MagnetSensor = new MagnetSensorConfigs()
        .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf)
        .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);

    inner.getConfigurator().apply(config);
  }

  /**
   * Gets the absolute position of the CANCoder in revolutions
   */
  public double getAbsolutePosition() {
    return inner.getAbsolutePosition().refresh().getValue();
  }
}
