package frc.robot.commands.locks;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector;


public class UnlockShooter extends Command {
    private final EndEffector endEffector;

    public UnlockShooter(EndEffector endEffector){
        this.endEffector = endEffector;    

    }

    @Override
    public void initialize() {
        endEffector.unlock();

    }

    @Override
    public boolean isFinished() {
        return true;

    }

}
