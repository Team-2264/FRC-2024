// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.motors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.NeoMotor;
import frc.robot.Constants.Swerve;
import frc.robot.Constants.Swerve.Mod2;
/**
 * square button press results in the clockwise movement of the neo motor
 * while triangle makes it stop.
 * get the velocity using the encoder and display it on the smart dashboard
 * set motor to move to a certain relative encoder value converted to degrees
 */

public class Neo {
  /** Creates a new Neo Subsystem. */
  private final CANSparkMax motor;
  private final RelativeEncoder encoder;
  private final SparkPIDController pidNeo;

  public Neo() {
    motor = new CANSparkMax(Constants.Swerve.Mod2.NeoMotorID, MotorType.kBrushless);
    encoder = motor.getEncoder();
    pidNeo = motor.getPIDController();
    resetPID();
    
    SmartDashboard.putNumber("Neo kP", Constants.NeoMotor.KP);
    SmartDashboard.putNumber("Neo kI", Constants.NeoMotor.KI);
    SmartDashboard.putNumber("Neo kD", Constants.NeoMotor.KD);
    
  }

  // Sets velocity in terms of -1.0 to 1.0
  public void setNeoSpeed(double speed) {
    // set the speed of the speed controller
    motor.set(speed);
    // speed : value btw -1.0 and 1.0
  }

  public void stopNeoMotor(){
    // stop motor
    motor.stopMotor();
  }


  // rotate the motor until it reachese the desired position
  // says it is going to be deprecated - just ignore it!!!!
  public void rotateNeoMotor(double degToRotate){
    double targetPosition =  encoder.getPosition() + degToRotate / 360.0;
    pidNeo.setReference(targetPosition, ControlType.kPosition);
  }

  // Sets the PID Values
  private void resetPID() {
    pidNeo.setP(SmartDashboard.getNumber("Neo kP", 0));
    pidNeo.setI(SmartDashboard.getNumber("Neo kI", 0));
    pidNeo.setD(SmartDashboard.getNumber("Neo kD", 0));
    pidNeo.setFF(0.0);
    pidNeo.setOutputRange(-0.25, 0.25);
    pidNeo.setIZone(0);
  }

}
