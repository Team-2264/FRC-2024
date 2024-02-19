// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.enums.HeldButton;

/**
 * Held buttons subsystem is used to store the most recent held button on the controller.
 *
 */
public class HeldButtons extends SubsystemBase {
  private HeldButton currentHeld;

  /** Creates a new HeldButtons. */
  public HeldButtons() {
    currentHeld = HeldButton.NONE;

  }

  public void setHeld(HeldButton held) {
    currentHeld = held;

  }

  public HeldButton currentHeld() {
    return currentHeld;

  }

}
