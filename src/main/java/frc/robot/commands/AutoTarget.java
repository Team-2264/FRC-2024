package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.ArmAngleEstimation;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.Swerve;

public class AutoTarget extends Command {
    private final Arm arm;
    private final Swerve swerve;

    private final Translation2d speakerTranslation = Constants.Targeting.speakerPose.get().getTranslation().toTranslation2d();

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
        final double distance_to_speaker = swerve.getPose().getTranslation().minus(speakerTranslation).getNorm();
        final ArmAngleEstimation angle_estimate = Constants.Targeting.getSpeakerArmAngle(distance_to_speaker);
        if(angle_estimate.inaccuracy < 0.1) {
            arm.shoulder.rotateTo(angle_estimate.estimate);
        }
   }

    @Override
    public void end(boolean interrupted) {
        swerve.unlockRotation();
    }
}
