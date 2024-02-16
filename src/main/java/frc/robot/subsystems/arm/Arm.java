package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.enums.ArmState;
import frc.robot.enums.IntakeStatus;

/**
 * Subystem for controlling the arm.
 * 
 */
public class Arm extends SubsystemBase {
    public final Shoulder shoulder;
    public final EndEffector endEffector;
    
    private ArmState state;

    /**
     * Constructs a new Arm instance.
     */
    public Arm() {
        shoulder = new Shoulder(Constants.Arm.neoConfigs);
        endEffector = new EndEffector(Constants.EndEffector.intakeNeoConfig, 
            Constants.EndEffector.shooterNeoConfigs,
            Constants.EndEffector.beamBreakPorts);

        state = ArmState.START;
        
    }

    /**
     * Sets the state of the arm. This is the main way to control the arm.
     * 
     * @param state
     */
    public void setState(ArmState state) {
        this.state = state;

        if (state != ArmState.CUSTOM) {
            setShoulderAngle(state.shoulderAngle());

        }

    }

    /**
     * Rotates the shoulder to a given angle.
     * 
     * @param angle The angle to rotate the shoulder to.
     */
    public void setShoulderAngle(double angle) {
        shoulder.rotateTo(angle);

    }

    /**
     * Spins up the shooter motors to a given speed.
     * Speed is a value between -1 and 1.
     * 
     * @param speed The speed to spin the motors at.
     */
    public void spinupShooter(double speed) {
        endEffector.spinupShooter(speed);

    }
    
    /**
     * Stops the shooter motors.
     */
    public void stopShooter() {
        endEffector.stopShooter();
    
    }

    /**
     * Starts the intake.
     */
    public void startIntake() {
        if (endEffector.intakeStatus() == IntakeStatus.STOPPED && !endEffector.hasNote()) {
            endEffector.intake(Constants.EndEffector.intakeSpeed);

        }

    }
    
    /**
     * Stops the intake.
     */
    public void stopIntake(){
        endEffector.stopIntake();
    }

    @Override 
    public void periodic() {
        shoulder.periodic();

        SmartDashboard.putNumber("Shoulder abs encoder", shoulder.absEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("Shoulder rel rots", shoulder.getRots());

        // Stop intaking if we have a note
        if (endEffector.intakeStatus() == IntakeStatus.INTAKING && endEffector.hasNote()) {
            endEffector.stopIntake();

        }

        // // Constatly update the angle of the shoulder to a target angle
        // if (state == ArmState.AUTO_SHOOT) {
        //     double distance = 0; // TODO: get distance from vision;
        //     int angle = (int)Constants.Targeting.getSpeakerArmAngle(distance);
        //     shoulder.rotateTo(angle);

        // }
        
    }

}
