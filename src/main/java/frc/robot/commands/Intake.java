package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector;
import frc.robot.Constants;
import frc.robot.enums.IntakeStatus;

public class Intake extends Command {
    private final EndEffector endEffector;

    public Intake(EndEffector endEffector){
        this.endEffector = endEffector;    

    }

    @Override
    public void initialize() {
        if (endEffector.intakeStatus() == IntakeStatus.STOPPED && !endEffector.hasNote()) {
            endEffector.intake(Constants.EndEffector.intakeSpeed);

        }

    }

    @Override
    public boolean isFinished() {
        return true;

    }

}
