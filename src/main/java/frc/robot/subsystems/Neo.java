// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * square button press results in the clockwise movement of the neo motor
 * while triangle makes it stop.
 */
public class Neo extends SubsystemBase {
  /** Creates a new Neo Subsystem. */
  private final CANSparkMax motor;

  public Neo() {
    motor = new CANSparkMax(Constants.Swerve.Mod2.NeoMotorID, MotorType.kBrushless);
  }

  // 
  public void runNeoMotor() {
    // set the speed of the speed controller
    motor.set(0.1);
    // speed : value btw -1.0 and 1.0
  }

  public void stopNeoMotor(){
    // stop motor
    motor.set(0);
  }

}
