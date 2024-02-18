// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.OptionalDouble;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants;
import frc.robot.enums.ArmState;
import frc.robot.subsystems.HeldButtons;
import frc.robot.subsystems.arm.Arm;

public class ChoiceState extends Command {
  private final Arm arm;
  private final Swerve swerve;
  private final HeldButtons heldButtons;

  /** Creates a new ChoiceState. */
  public ChoiceState(Arm arm, Swerve swerve, HeldButtons heldButtons) {
    this.arm = arm;
    this.swerve = swerve;
    this.heldButtons = heldButtons;

  }

  @Override
  public void initialize() {
    if (heldButtons.getHeld() == 1) { // Holding cross -> manual shoot
      arm.setState(ArmState.MANUAL_SHOOT);

    } else if (heldButtons.getHeld() == 2) { // Holding square -> amp
      arm.setState(ArmState.AMP);

    } else {
      Translation2d speakerTranslation = Constants.Targeting.speakerPose.getWpiBlue().getTranslation().toTranslation2d();
      swerve.lockOnto(speakerTranslation);
      arm.lockOnto(speakerTranslation);
      
    }

  } 

  @Override
  public boolean isFinished() {
    return true;
  }
}
