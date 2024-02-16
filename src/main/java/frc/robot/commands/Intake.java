package frc.robot.commands;

import frc.robot.subsystems.arm.Arm;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * A command for intaking.
 */
public class Intake extends Command {
    private final Arm arm;

    /**
     * Creates a new Intake command.
     * 
     * @param arm The arm subsystem to use.
     */
    public Intake(Arm arm){
        this.arm = arm;    
        addRequirements(arm);
        
    }

    @Override
    public void initialize() {
        arm.startIntake();
        
    }

    @Override
    public void end(boolean interrupted) {
        arm.stopIntake();

    }


    @Override
    public boolean isFinished() {
        return false;

    }

}
