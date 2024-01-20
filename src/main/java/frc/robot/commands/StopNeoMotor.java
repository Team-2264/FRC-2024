// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.Neo;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class StopNeoMotor extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private Neo NeoMotor;

//    @param subsystem
  public StopNeoMotor(Neo motor) {
    NeoMotor = motor;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(motor);
  }

  @Override
    public void initialize() {
        NeoMotor.stopNeoMotor();
    }

  @Override
  public boolean isFinished(){
    return true;
  }

  
}
