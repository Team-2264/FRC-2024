package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;

/**
 * Represents an individual swerve module that consists of an angle motor, a drive motor, and an angle encoder.
 */
public class SwerveModule {
    public int moduleNumber;

    private TalonFX angleMotor;
    private TalonFX driveMotor;
    private CANcoder angleEncoder;

    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    /**
     * Constructs a new SwerveModule instance.
     *
     * @param moduleNumber      The module's number or identifier.
     * @param driveMotorID      ID of the drive motor.
     * @param angleMotorID      ID of the angle motor.
     * @param angleEncoderID    ID of the angle encoder.
     * @param angleOffsetDouble Offset angle in degrees.
     * @param angleMotorInverted Whether the angle motor is inverted.
     * @param driveMotorInverted Whether the drive motor is inverted.
     */
    public SwerveModule(int moduleNumber, int driveMotorID, int angleMotorID, int angleEncoderID, double angleOffsetDouble, boolean angleMotorInverted, boolean driveMotorInverted) {
        this.moduleNumber = moduleNumber;

        this.angleOffset = Rotation2d.fromDegrees(angleOffsetDouble);

        // Angle Encoder Config - must be init before angle motor
        angleEncoder = new CANcoder(angleEncoderID);
        configAngleEncoder();

        // Drive Motor Config
        driveMotor = new TalonFX(driveMotorID);
        configDriveMotor(driveMotorInverted);

        // Angle Motor Config
        angleMotor = new TalonFX(angleMotorID);
        configAngleMotor(angleMotorInverted);

        lastAngle = getState().angle;

    }

    /**
     * Sets the desired state of the swerve module (angle and speed).
     *
     * @param desiredState The desired state of the module.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        desiredState = SwerveOptimizer.optimize(desiredState, getState().angle);

        setAngle(desiredState);
        setSpeed(desiredState);

    }

    /**
     * Sets the speed of the drive motor based on the desired state.
     *
     * @param desiredState The desired state of the module.
     */
    private void setSpeed(SwerveModuleState desiredState) {
        double velocity = Conversions.metersToRevs(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);

        driveMotor.setControl(
            new VelocityDutyCycle(velocity)
            .withFeedForward(feedForward.calculate(desiredState.speedMetersPerSecond))
        );

    }

    /**
     * Sets the angle of the swerve module based on the desired state.
     *
     * @param desiredState The desired state of the module.
     */
    private void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? lastAngle: desiredState.angle; // Prevent rotating module if speed is less then 1%. Prevents Jittering.

        angleMotor.setControl(new PositionDutyCycle(Conversions.degreesToRevs(angle.getDegrees(), Constants.Swerve.angleGearRatio)));

        lastAngle = angle;

    }

    /**
     * Gets the current angle of the swerve module.
     *
     * @return The current angle of the module.
     */
    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(Conversions.revsToDegrees(angleMotor.getPosition().refresh().getValue(), Constants.Swerve.angleGearRatio));

    }

    /**
     * Gets the absolute position of the angle encoder.
     *
     * @return The absolute angle of the encoder.
     */
    public Rotation2d getEncoder() {
        return Rotation2d.fromDegrees(Conversions.revsToDegrees(angleEncoder.getAbsolutePosition().refresh().getValue(), 1));
    }

    /**
     * Resets the angle motor's sensor position to an absolute value based on the encoder reading and the angle offset.
     */
    public void resetToAbsolute() {
        double absolutePosition = Conversions.degreesToRevs(getEncoder().getDegrees() - angleOffset.getDegrees(), Constants.Swerve.angleGearRatio);

        angleMotor.setPosition(absolutePosition);

        // Set the motor's target position to match the encoder position that we just set
        // angleMotor.setControl(new PositionDutyCycle(absolutePosition));
        
    }

    /**
     * Configures settings for the drive motor.
     *
     * @param driveMotorInverted Whether the drive motor should be inverted.
     */
    private void configDriveMotor(boolean driveMotorInverted) {
        driveMotor.getConfigurator().apply(new TalonFXConfiguration());
        driveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        driveMotor.setInverted(driveMotorInverted);
        driveMotor.setNeutralMode(Constants.Swerve.driveNeutralMode);
        driveMotor.setPosition(0); /// Could be bad

    }

    /**
     * Configures settings for the angle motor.
     *
     * @param angleMotorInverted Whether the angle motor should be inverted.
     */
    private void configAngleMotor(boolean angleMotorInverted) {
        angleMotor.getConfigurator().apply(new TalonFXConfiguration());
        angleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);
        angleMotor.setInverted(angleMotorInverted);
        angleMotor.setNeutralMode(Constants.Swerve.angleNeutralMode);
        resetToAbsolute();

    }

    /**
     * Configures settings for the angle encoder.
     */
    private void configAngleEncoder() {
        angleEncoder.getConfigurator().apply(new CANcoderConfiguration());
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCanCoderConfig);

    }

    /**
     * Gets the current state of the swerve module (velocity and angle).
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        double velocity = Conversions.revsToMeters(driveMotor.getVelocity().refresh().getValue(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.revsToDegrees(angleMotor.getPosition().refresh().getValue(), Constants.Swerve.angleGearRatio));

        return new SwerveModuleState(velocity, angle);

    }

    /**
     * Gets the current position of the swerve module (distance and angle).
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        double distance = Conversions.revsToMeters(driveMotor.getPosition().refresh().getValue(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
      
        return new SwerveModulePosition(distance, getAngle());
     
    }

}
