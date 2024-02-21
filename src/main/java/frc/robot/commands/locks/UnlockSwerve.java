package frc.robot.commands.locks;

import frc.robot.subsystems.Swerve;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;


public class UnlockSwerve extends Command {
    private final Swerve swerve;

    public UnlockSwerve(Swerve swerve){
        this.swerve = swerve;    

    }

    @Override
    public void initialize() {
        swerve.unlock();

    }

    @Override
    public boolean isFinished() {
        return true;

    }

}