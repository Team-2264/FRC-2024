package frc.robot.commands;

import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.Command;

public class ToggleTurbo extends Command {
    private final Swerve swerve;

    public ToggleTurbo(Swerve swerve){
        this.swerve = swerve;    
    }

    @Override
    public void initialize() {
        if (swerve.getTurboStatus()==true){
            swerve.turnTurboModeOn(false);
        }else{
            swerve.turnTurboModeOn(true);
        }
    }
}
