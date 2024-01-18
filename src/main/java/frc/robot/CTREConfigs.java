package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;


public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANcoderConfiguration swerveCanCoderConfig;

    public CTREConfigs(){
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANcoderConfiguration();

        /* Swerve Angle Motor Configurations */
        CurrentLimitsConfigs angleSupplyConfig = new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(Constants.Swerve.angleEnableCurrentLimit)
            .withSupplyCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit)
            .withSupplyCurrentThreshold(Constants.Swerve.anglePeakCurrentLimit)
            .withSupplyTimeThreshold(Constants.Swerve.anglePeakCurrentDuration);

        swerveAngleFXConfig.Slot0.kP = Constants.Swerve.angleKP;
        swerveAngleFXConfig.Slot0.kI = Constants.Swerve.angleKI;
        swerveAngleFXConfig.Slot0.kD = Constants.Swerve.angleKD;
        swerveAngleFXConfig.Slot0.kV = Constants.Swerve.angleKF; // API refer kF as kV
        swerveAngleFXConfig.CurrentLimits = angleSupplyConfig;
        

        // swerveAngleFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        // "The Talon FX and CANcoder sensors are always initialized to their absolute position in Phoenix 6." ??

        /* Swerve Drive Motor Configuration */
        CurrentLimitsConfigs driveSupplyConfig = new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(Constants.Swerve.driveEnableCurrentLimit)
            .withSupplyCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit)
            .withSupplyCurrentThreshold(Constants.Swerve.drivePeakCurrentLimit)
            .withSupplyTimeThreshold(Constants.Swerve.drivePeakCurrentDuration);

        swerveDriveFXConfig.Slot0.kP = Constants.Swerve.driveKP;
        swerveDriveFXConfig.Slot0.kI = Constants.Swerve.driveKI;
        swerveDriveFXConfig.Slot0.kD = Constants.Swerve.driveKD;
        swerveDriveFXConfig.Slot0.kV = Constants.Swerve.driveKF;        
        swerveDriveFXConfig.CurrentLimits = driveSupplyConfig;
        //swerveDriveFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        swerveDriveFXConfig.OpenLoopRamps = new OpenLoopRampsConfigs().withDutyCycleOpenLoopRampPeriod(Constants.Swerve.openLoopRamp);
        swerveDriveFXConfig.ClosedLoopRamps = new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(Constants.Swerve.closedLoopRamp);

        
        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.MagnetSensor = new MagnetSensorConfigs();
        swerveCanCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        swerveCanCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive; // Need to look at. //swerveCanCoderConfig.sensorDirection = Constants.Swerve.canCoderInvert;

        // swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        // swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;

    }

}
