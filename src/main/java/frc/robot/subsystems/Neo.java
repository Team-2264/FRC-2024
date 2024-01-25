// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * square button press results in the clockwise movement of the neo motor
 * while triangle makes it stop.
 * get the velocity using the encoder and display it on the smart dashboard
 * set motor to move to a certain relative encoder value converted to degrees
 */

public class Neo extends SubsystemBase {
  /** Creates a new Neo Subsystem. */
  private final CANSparkMax motor;
  private final RelativeEncoder encoder;
  private final SparkPIDController pidNeo;

  public Neo() {
    motor = new CANSparkMax(Constants.Swerve.Mod2.NeoMotorID, MotorType.kBrushless);
    encoder = motor.getEncoder();
    pidNeo = motor.getPIDController();
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
    double targetPosition = encoder.getPosition() + degToRotate * Constants.Swerve.encoderCountsPerDegree;
    pidNeo.setReference(targetPosition, ControlType.kPosition);
  }


  // still figuring out appropriate PID values
  public void controlPIDNeo(double kP, double kI, double kD) {
    pidNeo.setP(kP);
    pidNeo.setI(kI);
    pidNeo.setD(kD);
  }

  @Override
  public void periodic(){
    // update the smart dashboard with the proper encoder generated velocity
    // somehow uses the periodic function which i guess is cool
    SmartDashboard.putNumber("Encoder Velocity: ", encoder.getVelocity());
    SmartDashboard.putNumber("Encoder Position: ", encoder.getPosition());
  }

}
