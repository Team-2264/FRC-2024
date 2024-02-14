package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.lib.motors.Neo;
import frc.lib.motors.NeoConfiguration;
import frc.robot.enums.IntakeStatus;

/**
 * Subystem for controlling the end effector.
 * 
 */
public class EndEffector {
    private Neo intakeMoter;
    private IntakeStatus intakeStatus = IntakeStatus.STOPPED;

    private Neo[] shooterMotors;

    private DigitalInput intakeBeam;

    /**
     * Constructs a new EndEffector instance.
     * 
     * @param intakeMotorID The ID of the intake motor.
     * @param shooterMotorIDs The IDs of the shooter motors.
     */
    public EndEffector(NeoConfiguration intakeNeoConfig, NeoConfiguration[] shooterNeoConfigs, int beamBreakPort) {
        // create motors
        intakeMoter = new Neo(intakeNeoConfig);
        shooterMotors = new Neo[shooterNeoConfigs.length];
        for (int i = 0; i < shooterNeoConfigs.length; i++) {
            shooterMotors[i] = new Neo(shooterNeoConfigs[i]);
        }

        // create beam
        intakeBeam = new DigitalInput(beamBreakPort);
        
    }

    /**
     * Spins up the shooter motors to a given speed.
     * Speed is a value between -1 and 1.
     */
    public void spinupShooter(double speed) {
        for (Neo motor : shooterMotors) {
            motor.rotateAtSpeed(speed);
        }
    }

    /**
     * Stops the shooter motors.
     */
    public void stopShooter() {
        for (Neo motor : shooterMotors) {
            motor.stop();

        }

    }

    /**
     * Starts the intake.
     */
    public void startIntake() {
        intakeMoter.rotateAtSpeed(0.6);
        intakeStatus = IntakeStatus.INTAKING;

    }


    /**
     * Reverses the intake. Sometimes referred to as "outtaking".
     */
    public void reverseIntake() {
        intakeMoter.rotateAtSpeed(-0.3);
        intakeStatus = IntakeStatus.OUTTAKING;

    }

    /**
     * Stops the intake motor.
     */
    public void stopIntake() {
        intakeMoter.rotateAtSpeed(0);
        intakeStatus = IntakeStatus.STOPPED;

    }

    /**
     * Returns the state of the intake.
     * @return The state of the intake.
     */
    public IntakeStatus intaking() {
        return intakeStatus;
    }

    /** 
     * Returns the state of the intakes beam.
     * @return The state of the intake beam.
     */
    public boolean hasNote() {
        return !intakeBeam.get();

    }

}
