// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.motors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

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

  /**
   * Creates a Neo motor with the specified configuration
   */
  public Neo(NeoConfiguration c) {
    motor = new CANSparkMax(c.canId, MotorType.kBrushless);
    encoder = motor.getEncoder();
    pidNeo = motor.getPIDController();

    pidNeo.setP(c.kP);
    pidNeo.setI(c.kI);
    pidNeo.setD(c.kD);
    pidNeo.setFF(0.0);
    pidNeo.setOutputRange(c.outputRangeLower, c.outputRangeUpper);
    pidNeo.setIZone(0);
  }

  /**
   * Starts rotating the motor at a specific speed.
   * @param speed The speed in the range of -1 to 1.
   */
  public void rotateAtSpeed(double speed) {
    motor.set(speed);
  }

  /**
   * Stops rotating the motor.
   */
  public void stop(){
    motor.stopMotor();
  }

  /**
   * Gets the position of the motor. This is reset to 0 when the robot is turned on.
   * @return The current rotation of the motor in rotations.
   */
  public double getPosition() {
    return encoder.getPosition();
  }

  /**
   * Rotates the motor to a specific position.
   * @param position The angle to rotate to in rotations
   */
  public void rotateTo(double position){
    pidNeo.setReference(position, ControlType.kPosition);
  }
}
