// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.enums.ArmState;
import frc.robot.subsystems.HeldButtons;
import frc.robot.subsystems.arm.Arm;

public class ChoiceShoot extends Command {
  private final Arm arm;
  private final HeldButtons heldButtons;

  /** Creates a new Test. */
  public ChoiceShoot(Arm arm, HeldButtons heldButtons) {
    this.arm = arm;
    this.heldButtons = heldButtons;

    addRequirements(arm);
    addRequirements(heldButtons);
    // Use addRequirements() here to declare subsystem dependencies.

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (heldButtons.getHeld() == 1) {
      arm.spinupShooter(1);

    } else if (heldButtons.getHeld() == 2) {
      arm.spinupShooter(0.2);

    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
