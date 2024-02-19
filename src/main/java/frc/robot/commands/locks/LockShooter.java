package frc.robot.commands.locks;

import frc.robot.Constants;
import frc.robot.subsystems.EndEffector;

import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;


public class LockShooter extends Command {
    private final EndEffector endEffector;

    public LockShooter(EndEffector endEffector){
        this.endEffector = endEffector;    

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
        
        endEffector.lockOnto(speakerTranslation);

    }

    @Override
    public boolean isFinished() {
        return true;

    }

}
