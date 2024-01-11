// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.util.Units;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other purpose.
 * All constants should be declared globally (i.e. public static). 
 * Do not put anything functional in this class.
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    /**
     * Operator class holds constants related to operator input.
     */
    public static class Operator {
        public static final int controllerPort = 0;
        public static final double stickDeadband = 0.1;

    }

    public static final class LedStrip {
        public static final int pwmPort = 9;
        public static final int numLeds = 10;
        public static final float scaleFactor = 0.5f;
    }

    /**
     * Swerve class holds constants related to the swerve drive system.
     */
    public static final class Swerve {
        public static final int pigeonID = 15;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        // Drivetrain Constants
        public static final double trackWidth = Units.inchesToMeters(25.5);
        public static final double wheelBase = Units.inchesToMeters(30);
        public static final double wheelDiameter = Units.inchesToMeters(4.05);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double openLoopRamp = 0.5;
        public static final double closedLoopRamp = 0.5;

        public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1
        public static final double angleGearRatio = (12.8 / 1.0); // 12.8:1

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)

        );

        // Swerve Current Limiting
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        // Angle Motor PID Values
        public static final double angleKP = 0.6;
        public static final double angleKI = 0.0;
        public static final double angleKD = 12.0;
        public static final double angleKF = 0.0;

        // Drive Motor PID Values
        public static final double driveKP = 0.10;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        // Drive Motor Characterization Values
        public static final double driveKS = (0.667 / 12); // divide by 12 to convert from volts to percent
                                                           // output for
                                                           // CTRE
        public static final double driveKV = (2.44 / 12);
        public static final double driveKA = (0.27 / 12);

        // Swerve Profiling Values
        public static final double maxSpeed = 1; // meters per second
        public static final double maxAngularVelocity = Math.PI * 1.5; // radians per second (rad/s)

        // Neutral Modes
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        // Global Motor Inverts
        public static final boolean driveMotorInvert = false;
        public static final boolean angleMotorInvert = false;

        // Angle Encoder Invert
        public static final boolean canCoderInvert = false;

        // Module Specific Constants
        // Front Left Module - Module 0
        public static final class Mod0 {
            public static final int driveMotorID = 40;
            public static final int angleMotorID = 41;
            public static final int angleEncoderID = 48;

            public static final double angleOffset = 203.994;
            public static final boolean angleInverted = false;
            public static final boolean driveInverted = false;

        }

        // Front Right Module - Module 1
        public static final class Mod1 {
            public static final int driveMotorID = 42;
            public static final int angleMotorID = 43;
            public static final int angleEncoderID = 49;

            public static final double angleOffset = 206.367;
            public static final boolean angleInverted = false;
            public static final boolean driveInverted = false;

        }

        // Back Left Module - Module 2
        public static final class Mod2 {
            public static final int driveMotorID = 46;
            public static final int angleMotorID = 47;
            public static final int angleEncoderID = 50;

            public static final double angleOffset = 330.645;

            public static final boolean angleInverted = false;
            public static final boolean driveInverted = false;

        }

        // Back Right Module - Module 3
        public static final class Mod3 {
            public static final int driveMotorID = 44;
            public static final int angleMotorID = 45;
            public static final int angleEncoderID = 51;

            public static final double angleOffset = 125.596;

            public static final boolean angleInverted = false;
            public static final boolean driveInverted = false;

        }

    }

}
