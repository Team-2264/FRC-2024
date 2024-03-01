package frc.robot.commands;

import frc.robot.subsystems.EndEffector;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * A command for feeding the shooter.
 */
public class FeedShooter extends Command {
    private final EndEffector endEffector;
    private double startTime;

    private double speed;

    /**
     * Creates a new FeedShooter command.
     * 
     * @param arm The arm subsystem to use.
     */
    public FeedShooter(EndEffector endEffector, double speed){
        this.endEffector = endEffector;    
        this.speed = speed;

    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();

        endEffector.feed(speed);
        
    }

    @Override
    public void end(boolean interrupted) {
        endEffector.stopIntake();

    }


    @Override
    public boolean isFinished() {
        return (Timer.getFPGATimestamp() - startTime) > 2.5;

    }

}
