// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;

public class StartShoot extends Command {
  private final Arm arm;
  
  /** Creates a new StartShoot. */
  public StartShoot(Arm arm) {
    this.arm = arm;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.spinupShooter(1);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
