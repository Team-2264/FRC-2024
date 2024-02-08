package frc.robot.commands;

import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * A command for toggling turbo mode.
 */
public class ToggleTurbo extends Command {
    private final Swerve swerve;

    public ToggleTurbo(Swerve swerve){
        this.swerve = swerve;    
    }

    @Override
    public void initialize() {
        if (swerve.getTurboStatus()==true){
            swerve.toggleTurboMode(false);
        }else{
            swerve.toggleTurboMode(true);
        }
    }

    @Override
    public boolean isFinished() {
        return true;

    }

}