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

    private DigitalInput[] intakeBeams;

    /**
     * Constructs a new EndEffector instance.
     * 
     * @param intakeMotorID The ID of the intake motor.
     * @param shooterMotorIDs The IDs of the shooter motors.
     */
    public EndEffector(NeoConfiguration intakeNeoConfig, NeoConfiguration[] shooterNeoConfigs, int[] beamBreakPorts) {
        // create motors
        intakeMoter = new Neo(intakeNeoConfig);
        shooterMotors = new Neo[shooterNeoConfigs.length];
        for (int i = 0; i < shooterNeoConfigs.length; i++) {
            shooterMotors[i] = new Neo(shooterNeoConfigs[i]);
        }

        // create beams
        intakeBeams = new DigitalInput[2];
        for (int i = 0; i < beamBreakPorts.length; i++) {
            intakeBeams[i] = new DigitalInput(beamBreakPorts[i]);

        }
        
    }

    /**
     * Spins up the shooter motors to a given speed.
     * Speed is a value between -1 and 1.
     */
    public void spinupShooter(double speed) {
        shooterMotors[0].rotateAtSpeed(speed);
    }

    /**
     * Stops the shooter motors.
     */
    public void stopShooter() {
        shooterMotors[0].stop();
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
     * Feeds a held note into the shooter.
     */
    public void feed() {
        intakeMoter.rotateAtSpeed(1);
        intakeStatus = IntakeStatus.FEEDING;

    }

    public void slowFeed() {
        intakeMoter.rotateAtSpeed(0.3);
        intakeStatus = IntakeStatus.FEEDING;


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
     * Returns the state of the intakes beam.
     * @return The state of the intake beam. 
     */
    public boolean hasNote() {
        return intakeBeams[0].get() || intakeBeams[1].get(); 

    }

}
