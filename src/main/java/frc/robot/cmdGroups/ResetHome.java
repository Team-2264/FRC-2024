package frc.robot.cmdGroups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.locks.UnlockArm;
import frc.robot.commands.locks.UnlockShooter;
import frc.robot.commands.locks.UnlockSwerve;
import frc.robot.enums.ArmState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.EndEffector;

public class ResetHome extends SequentialCommandGroup {
    public ResetHome(Arm arm, Swerve swerve, EndEffector endEffector) {
        addCommands(
            new UnlockArm(arm),
            new UnlockSwerve(swerve),
            new UnlockShooter(endEffector),
            new InstantCommand(() -> endEffector.stopIntake()),
            new InstantCommand(() -> endEffector.stopShooter()),
            new InstantCommand(() -> arm.setState(ArmState.HOME))
            
        );

    }
    
}
