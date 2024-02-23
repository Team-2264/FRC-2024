package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.motors.Neo;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.enums.IntakeStatus;
import frc.robot.enums.ShooterStatus;

/**
 * Subystem for controlling the end effector.
 * 
 */
public class EndEffector extends SubsystemBase {
    private Neo intakeMoter;
    private IntakeStatus intakeStatus = IntakeStatus.STOPPED;

    private Neo[] shooterMotors;
    private ShooterStatus shooterStatus = ShooterStatus.STOPPED;

    private DigitalInput[] intakeBeams;

    private Optional<Translation2d> lockedOnto = Optional.empty();

    private RobotContainer container;

    /**
     * Constructs a new EndEffector instance.
     * 
     */
    public EndEffector(RobotContainer container) {
        // create motors
        intakeMoter = new Neo(Constants.EndEffector.intakeNeoConfig);
        shooterMotors = new Neo[2];
        for (int i = 0; i < 2; i++) {
            shooterMotors[i] = new Neo(Constants.EndEffector.shooterNeoConfigs[i]);
        }

        // create beams
        intakeBeams = new DigitalInput[2];
        for (int i = 0; i < 2; i++) {
            intakeBeams[i] = new DigitalInput(Constants.EndEffector.beamBreakPorts[i]);

        }

        this.container = container;
        
    }

    
    public void lockOnto(Translation2d translation2d) {
        lockedOnto = Optional.of(translation2d);
        shooterStatus = ShooterStatus.LOCKED;
    }

    public void unlock() {
        lockedOnto = Optional.empty();
        shooterStatus = ShooterStatus.STOPPED;
        
    }

    public boolean locked() {
        return lockedOnto.isPresent();

    }

    /**
     * Spins up the shooter motors to a given speed.
     * Speed is a value between -1 and 1.
     */
    public void spinupShooter(double speed) {
        shooterMotors[0].rotateAtSpeed(speed);
        shooterStatus = ShooterStatus.SPINNING;
    }

    /**
     * Stops the shooter motors.
     */     
    public void stopShooter() {
        shooterMotors[0].stop();
        shooterStatus = ShooterStatus.STOPPED;
    }
    
    /**
     * Starts the intake.
     * @param speed The speed to intake at from 0 to 1.
     */
    public void intake(double speed) {
        intakeMoter.rotateAtSpeed(speed);
        intakeStatus = IntakeStatus.INTAKING;

    }
    
    /**
     * Reverses the intake. Referred to as "outtaking".
     * @param speed The speed to outtake at from 0 to 1.
     */
    public void outtake(double speed) {
        intakeMoter.rotateAtSpeed(-speed);
        intakeStatus = IntakeStatus.OUTTAKING;
    }

    /**
     * Feeds a held note into the shooter.
     */
    public void feed() {
        intakeMoter.rotateAtSpeed(1);
        intakeStatus = IntakeStatus.FEEDING;

    }

    /**
     * Stops the intake motor.
     */
    public void stopIntake() {
        intakeMoter.stop();
        intakeStatus = IntakeStatus.STOPPED;

    }

    /**
     * Returns the state of the intake.
     * @return The state of the intake.
     */
    public IntakeStatus intakeStatus() {
        return intakeStatus;
    }

    /**
     * Returns the state of the shooter.
     * @return The state of the shooter.
     */
    public ShooterStatus shooterStatus() {
        return shooterStatus;

    }

    /** 
     * Returns the state of the intakes beam.
     * @return The state of the intake beam. 
     */
    public boolean hasNote() {
        return intakeBeams[0].get() || intakeBeams[1].get(); 

    }

    public double shooterSpeed() {
        return (shooterMotors[0].getVelocity() + shooterMotors[1].getVelocity())/2.0;
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Intake Status", intakeStatus.toString());
        SmartDashboard.putString("Shooter Status", shooterStatus.toString());

        SmartDashboard.putNumber("Shooter speed: ", shooterSpeed());

        SmartDashboard.putBoolean("Note", hasNote());

        // Stop intaking if we have a note
        if (intakeStatus == IntakeStatus.INTAKING && hasNote()) {
            stopIntake();

        }

        // If we are locked onto a speaker, calculate the flywheel speed.
        if (lockedOnto.isPresent()) {
            double distance_to_speaker = container.swerve.getPose().getTranslation().minus(lockedOnto.get()).getNorm();

            shooterMotors[0].rotateAtSpeed(Constants.Targeting.getFlywheelSpeed(distance_to_speaker));

        }

    }

}
