// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.motors.NeoConfiguration;

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

    /**
     *  EndEffector class holds constants related to the end effector.
     */
    public static class EndEffector {
        public static final int intakeMotorID = 0;
        public static final NeoConfiguration[] shooterNeoConfigs = new NeoConfiguration[] {
            new NeoConfiguration(0)
                .withKP(0.0)
                .withKI(0.0)
                .withKD(0.0),
            new NeoConfiguration(1)
                .withKP(0.0)
                .withKI(0.0)
                .withKD(0.0)
               
        };

        public static final int beamBreakPort = 0;
        
        public static final double intakeSpeed = 0.5;

    }

    public static class Vision {
        public static final String cameraName = "apriltag";
        
        // Robot to Camera Transform
        public static final Transform3d robotToCamera = new Transform3d(
            new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)),
            new Rotation3d(0, 0, 0)

        );

    }

    /**
     * LedStrip class holds constants related to the LED strip.
     */
    public static final class LedStrip {
        public static final int pwmPort = 9;
        public static final int numLeds = 10;
        public static final float scaleFactor = 0.5f;
    }

    public static final class Arm {
        public static final NeoConfiguration[] motorConfigs = new NeoConfiguration[] {
            new NeoConfiguration(8),
            new NeoConfiguration(0),
            new NeoConfiguration(8),
            new NeoConfiguration(0)
        };
        public static final double KP = 0.5;
        public static final double KI = 0.001;
        public static final double KD = 0;
        public static final int angleEncoderID = 49;
        public static final double shoulderRatio = -12.76 * 52/18 * 64/10;
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
        public static final double angleKP = 1.201173; // 0.6
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.024023; // 12.0
        public static final double angleKF = 0.0; // Kf is Kv in CTRE v6 API

        // Drive Motor PID Values
        public static final double driveKP = 0.20019; //0.1
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0; // Kf is Kv in CTRE v6 API

        // Drive Motor Characterization Values
        public static final double driveKS = (0.667 / 12); // divide by 12 to convert from volts to percent
                                                           // output for
                                                           // CTRE
        public static final double driveKV = (2.44 / 12);
        public static final double driveKA = (0.27 / 12);

        // Swerve Profiling Values
        public static final double maxSpeed = 1;  // meters per second
        public static final double maxAngularVelocity = Math.PI * 1.5; // radians per second (rad/s)

        // Neutral Modes
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        // Global Motor Inverts
        public static final boolean driveMotorInvert = false;
        public static final boolean angleMotorInvert = false;

        // Global Angle Encoder Inverts
        public static final SensorDirectionValue canCoderInvert = SensorDirectionValue.CounterClockwise_Positive;

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

        // Autonomous 
        public static final HolonomicPathFollowerConfig pathFollowerConfig = 
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                new PIDConstants(0.220019, 0.0, 0.0), // Translation PID constants
                new PIDConstants(1.201173, 0.0, 0.024023), // Rotation PID constants
                4.5, // Max module speed, in m/s
                0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                new ReplanningConfig() // Default path replanning config. See the API for the options here
            );

    }


}
