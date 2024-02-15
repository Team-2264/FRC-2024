package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;

public class MoveArmBy extends Command {
    Arm arm;
    double offset;

    public MoveArmBy(Arm arm, double offset) {
        this.arm = arm;
        this.offset = offset;
    }

    @Override
    public void initialize() {
        arm.moveShoulder(offset);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
