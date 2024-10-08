package frc.robot.commands.locks;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

/**
* Command to allow movement of arm based on ArmState(enum)
*/

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
