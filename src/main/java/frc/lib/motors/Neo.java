// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.motors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.ExternalFollower;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

  /** Creates a new Neo Subsystem. */
public class Neo {
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

    if(c.breakMode) {
        motor.setIdleMode(IdleMode.kBrake);
    } else {
        motor.setIdleMode(IdleMode.kCoast);
    }

    pidNeo.setP(c.kP);
    pidNeo.setI(c.kI);
    pidNeo.setD(c.kD);
    pidNeo.setFF(0.0);
    pidNeo.setOutputRange(c.outputRangeLower, c.outputRangeUpper);
    pidNeo.setIZone(0);

    encoder.setVelocityConversionFactor(1.0/60.0);

    if(c.followMotor.isPresent()) {
        motor.follow(ExternalFollower.kFollowerSpark, c.followMotor.getAsInt(), c.followInvert);
    }
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
   * Gets the velocity of the motor.
   * @return The current velocity of the motor in rotations per second.
   */
  public double getVelocity() {
    return encoder.getVelocity();
  }

  /**
   * Rotates the motor to a specific position relative to where it started.
   * @param position The angle to rotate to in rotations
   */
  public void rotateTo(double position){
    pidNeo.setReference(position, ControlType.kPosition);
  }
  
}
