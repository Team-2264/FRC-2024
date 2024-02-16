package frc.robot.commands;

import frc.robot.subsystems.arm.Arm;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * A command for feeding the shooter.
 */
public class FeedShooter extends Command {
    private final Arm arm;
    private double startTime;

    /**
     * Creates a new FeedShooter command.
     * 
     * @param arm The arm subsystem to use.
     */
    public FeedShooter(Arm arm){
        this.arm = arm;    

    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();

        arm.endEffector.feed();
        
    }

    @Override
    public void end(boolean interrupted) {
        arm.stopIntake();

    }


    @Override
    public boolean isFinished() {
        return (Timer.getFPGATimestamp() - startTime) > 1;

    }

}
