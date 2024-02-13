package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.lib.motors.TalonFx;
import frc.lib.motors.CanCoder;

/**
 * Represents an individual swerve module that consists of an angle motor, a drive motor, and an angle encoder.
 */
public class SwerveModule {
    public int moduleNumber;

    private TalonFx angleMotor;
    private TalonFx driveMotor;
    private CanCoder angleEncoder;

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
    public SwerveModule(int moduleNumber, int driveMotorID, int angleMotorID, int angleEncoderID, double angleOffsetDouble) {
        this.moduleNumber = moduleNumber;

        this.angleOffset = Rotation2d.fromDegrees(angleOffsetDouble);

        // Angle Encoder Config - must be init before angle motor
        angleEncoder = new CanCoder(angleEncoderID);

        // Drive Motor Config
        driveMotor = new TalonFx(driveMotorID).withConfiguration(Constants.Swerve.driveMotorConfig);
        driveMotor.setSensorPosition(0);

        // Angle Motor Config
        angleMotor = new TalonFx(angleMotorID).withConfiguration(Constants.Swerve.angleMotorConfig);
        resetToAbsolute();

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

        driveMotor.rotateWithVelocity(velocity, feedForward.calculate(desiredState.speedMetersPerSecond));
    }

    /**
     * Sets the angle of the swerve module based on the desired state.
     *
     * @param desiredState The desired state of the module.
     */
    private void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? lastAngle: desiredState.angle; // Prevent rotating module if speed is less then 1%. Prevents Jittering.

        angleMotor.rotateTo(Conversions.degreesToRevs(angle.getDegrees(), Constants.Swerve.angleGearRatio));

        lastAngle = angle;
    }

    /**
     * Gets the current angle of the swerve module.
     *
     * @return The current angle of the module.
     */
    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(Conversions.revsToDegrees(angleMotor.getPosition(), Constants.Swerve.angleGearRatio));

    }

    /**
     * Gets the absolute position of the angle encoder.
     *
     * @return The absolute angle of the encoder.
     */
    public Rotation2d getEncoder() {
        return Rotation2d.fromDegrees(Conversions.revsToDegrees(angleEncoder.getAbsolutePosition(), 1));
    }

    /**
     * Resets the angle motor's sensor position to an absolute value based on the encoder reading and the angle offset.
     */
    public void resetToAbsolute() {
        double absolutePosition = Conversions.degreesToRevs(getEncoder().getDegrees() - angleOffset.getDegrees(), Constants.Swerve.angleGearRatio);

        angleMotor.setSensorPosition(absolutePosition);
    }

    /**
     * Gets the current state of the swerve module (velocity and angle).
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        double velocity = Conversions.revsToMeters(driveMotor.getVelocity(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.revsToDegrees(angleMotor.getPosition(), Constants.Swerve.angleGearRatio));

        return new SwerveModuleState(velocity, angle);

    }

    /**
     * Gets the current position of the swerve module (distance and angle).
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        double distance = Conversions.revsToMeters(driveMotor.getPosition(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
      
        return new SwerveModulePosition(distance, getAngle());
     
    }

}
