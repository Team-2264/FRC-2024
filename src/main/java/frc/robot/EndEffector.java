package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;

public class EndEffector {
    private TalonFX intakeMoter;
    private boolean intaking = false;

    private TalonFX[] shooterMotors;

    private DigitalInput intakeBeam;

    /**
     * Constructs a new EndEffector instance.
     * 
     * @param intakeMotorID The ID of the intake motor.
     * @param shooterMotorIDs The IDs of the shooter motors.
     */
    public EndEffector(int intakeMotorID, int[] shooterMotorIDs) {
        // create motors
        intakeMoter = new TalonFX(intakeMotorID);
        shooterMotors = new TalonFX[shooterMotorIDs.length];
        for (int i = 0; i < shooterMotorIDs.length; i++) {
            shooterMotors[i] = new TalonFX(shooterMotorIDs[i]);
        }

        // create beam
        intakeBeam = new DigitalInput(0);
        
    }

    /**
     * Starts the intake.
     * @param speed
     */
    public void startIntake() {
        intakeMoter.set(0.1);
        intaking = true;

    }

    /**
     * Stops the intake motor.
     */
    public void stopIntake() {
        intakeMoter.set(0);
        intaking = false;
    }

    /**
     * Returns the state of the intake.
     * @return The state of the intake.
     */
    public boolean intaking() {
        return intaking;
    }

    /** 
     * Returns the state of the intakes beam.
     */
    public boolean hasNote() {
        return !intakeBeam.get();

    }

}
