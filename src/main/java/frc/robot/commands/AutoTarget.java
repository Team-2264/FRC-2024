package frc.robot.commands;

import java.util.OptionalDouble;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.Swerve;

public class AutoTarget extends Command {
    private final Arm arm;
    private final Swerve swerve;

    private final Translation2d speakerTranslation = Constants.Targeting.speakerPose.getWpiBlue().getTranslation().toTranslation2d();

    public AutoTarget(Arm arm, Swerve swerve) {
        this.arm = arm;
        this.swerve = swerve;
    }

    @Override
    public void initialize() {
        swerve.lockOnto(speakerTranslation);
    }

    @Override
    public void execute() {
        // final double distance_to_speaker = swerve.getPose().getTranslation().minus(speakerTranslation).getNorm();
        // final OptionalDouble angle_estimate = Constants.Targeting.getSpeakerArmAngle(distance_to_speaker);
        // if(angle_estimate.isPresent()) {
        //     arm.shoulder.rotateTo(angle_estimate.getAsDouble());
        // }

   }

    @Override
    public void end(boolean interrupted) {
        swerve.unlockRotation();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
