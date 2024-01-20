// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import frc.robot.subsystems.Neo;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class RunMotor extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private Neo NeoMotor;

//    @param subsystem
  public RunMotor(Neo motor) {
    NeoMotor
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(motor);
  }

  @Override
    public void initialize() {
        NeoMotor.runNeoMotor();
    }

  @Override
  public boolean isFinished(){
    return true;
  }

  
}
