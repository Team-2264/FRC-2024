package frc.robot.commands;

import frc.robot.enums.ArmState;
import frc.robot.subsystems.arm.Arm;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * A command for setting the arm to a specific angle.
 * 
 */
public class SetArmAngle extends Command {
    private final Arm arm;
    private final int angle;

    public SetArmAngle(Arm arm, int angle) {
        this.arm = arm;    
        addRequirements(arm);

        this.angle = angle;

    }

    @Override
    public void initialize() {
        arm.setState(ArmState.CUSTOM);
        arm.setShoulderAngle(angle);
       
    }

    @Override
    public boolean isFinished() {
        return true;

    }

}
