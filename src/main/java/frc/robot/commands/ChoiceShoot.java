// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HeldButtons;
import frc.robot.subsystems.arm.Arm;

public class ChoiceShoot extends Command {
  private final Arm arm;
  private final HeldButtons heldButtons;

  /** Creates a new ChoiceShoot. */
  public ChoiceShoot(Arm arm, HeldButtons heldButtons) {
    this.arm = arm;
    this.heldButtons = heldButtons;

  }

  @Override
  public void initialize() {
    if (heldButtons.getHeld() == 1) { // Holding cross -> manual shoot
      arm.spinupShooter(0.6);
 
    } else if (heldButtons.getHeld() == 2) { // Holding square -> amp
      arm.spinupShooter(0.2);

    } else {  
       arm.spinupShooter(0.9);

    }

  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
