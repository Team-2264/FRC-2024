package frc.robot.commands.locks;

import frc.robot.subsystems.arm.Arm;

import edu.wpi.first.wpilibj2.command.Command;


public class UnlockArm extends Command {
    private final Arm arm;

    public UnlockArm(Arm arm){
        this.arm = arm;    

    }

    @Override
    public void initialize() {
        arm.unlock();

    }

    @Override
    public boolean isFinished() {
        return true;

    }

}
