// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Held buttons subsystem is used to store the current held button on the controller. This is used to determine the outcome of some commands.
 *
 */
public class HeldButtons extends SubsystemBase {
  private int heldbutton;

  /** Creates a new HeldButtons. */
  public HeldButtons() {
    heldbutton = 0;

  }

  public void setHeld(int held) {
    heldbutton = held;

  }

  public int getHeld() {
    return heldbutton;

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("HELD", heldbutton);

  }
}
