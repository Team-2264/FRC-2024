// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climbing;

public class Descend extends Command {
  private final Climbing climbing;

  /** Creates a new Accend. */
  public Descend(Climbing climbing) {
    this.climbing = climbing;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climbing.descend(Constants.Climbing.descendSpeed);

  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climbing.stopWinch();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  
}
