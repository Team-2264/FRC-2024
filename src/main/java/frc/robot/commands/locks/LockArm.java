package frc.robot.commands.locks;

import frc.robot.Constants;
import frc.robot.enums.ArmState;
import frc.robot.subsystems.Arm;

import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;


public class LockArm extends Command {
    private final Arm arm;

    public LockArm(Arm arm){
        this.arm = arm;    

    }

    @Override
    public void initialize() {
        Optional<Alliance> alliance2 = DriverStation.getAlliance();
        if(alliance2.isEmpty()) {
            throw new RuntimeException("Failed to get alliance: Alliance is not set in Driverstation");
        }

        Alliance alliance = alliance2.get();
        Translation2d speakerTranslation = switch (alliance) {
            case Blue -> Constants.Targeting.blueSpeakerPose.getWpiBlue().getTranslation().toTranslation2d();
            case Red -> Constants.Targeting.redSpeakerPose.getWpiBlue().getTranslation().toTranslation2d();

        };
        
        arm.lockOnto(speakerTranslation);
        arm.setState(ArmState.LOCKED);

    }

    @Override
    public boolean isFinished() {
        return true;

    }

}
