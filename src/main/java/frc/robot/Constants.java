// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.lib.FieldPose;
import frc.lib.motors.NeoConfiguration;
import frc.lib.motors.TalonFxConfiguration;

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
     * Targeting class holds constants related to targeting.
     */
    public static final class Targeting {
        public static final double armLength = 1; // meters

        public static final double targetHeight = 2.5;

         /** 
         *  Get the arm angle to hit the speaker
         *  Visualiation: https://www.desmos.com/calculator/zvokkhmnqa
         * 
         *  @param targetDistance The distance to the speaker
         *  @return The angle to hit the speaker in degrees
         * 
         */
        public static final double getSpeakerArmAngle(double targetDistance) {
            double numerator = Math.sqrt(-(armLength * armLength) + (targetHeight * targetHeight) + (targetDistance * targetDistance)) + targetHeight;
            double angle = Math.atan(numerator / (armLength + targetDistance));

            return Math.toDegrees(angle);
        }

    }
    
    /**
     * Arm class holds constants related to the arm.
     */
    public static final class Arm {
        public static final NeoConfiguration[] neoConfigs = new NeoConfiguration[] {
            new NeoConfiguration(20)
                .withBrakeMode(true)
                .withKP(1.0)
                .withKI(0.0)
                .withKD(0.1),
            new NeoConfiguration(21)
                .withBrakeMode(true)
                .followMotor(20)
                .withFollowInvert(true),
            new NeoConfiguration(22)
                .withBrakeMode(true)
                .followMotor(20)
                .withFollowInvert(true),
            new NeoConfiguration(23)
                .withBrakeMode(true)
                .followMotor(20)
        };

        public static ArmFeedforward shoulderFeedForward = new ArmFeedforward(
            0.0, // kS 
            0.0, // kG 
            0.0, // kV 
            0.0  // kA 
        );

        public static PIDController shoulderFeedback = new PIDController(
            0.0, // kP
            0.0, // kI
            0.0  // kD
        );

        public static final double shoulderMaxPower = 0.05;
        
        public static final int angleEncoderID = 1;

        public static final double shoulderRatio = 235.8276644 / 1; // not sure if this is correct

        public static final double shoulderOffset = 0.4178160104454; // rotations

    }


    /**
     *  EndEffector class holds constants related to the end effector.
     */
    public static class EndEffector {
        public static final int intakeMotorID = 0;

        public static final NeoConfiguration intakeNeoConfig = new NeoConfiguration(26)
            .withKP(0.0)
            .withKI(0.0)
            .withKD(0.0);

        public static final NeoConfiguration[] shooterNeoConfigs = new NeoConfiguration[] {
            new NeoConfiguration(24)
                .withKP(0.0)
                .withKI(0.0)
                .withKD(0.0),
            new NeoConfiguration(25)
                .withKP(0.0)
                .withKI(0.0)
                .withKD(0.0)
                .followMotor(24)
               
        };

        public static final int beamBreakPort = 0;
        
        public static final double intakeSpeed = 0.6;

    }

    /**
     * Vision class holds constants related to the vision system.
     */
    public static class Vision {
        public static final String cameraName = "apriltag";

        public static final Matrix<N3, N1> visionStandardDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30));
        public static double singleTargetMultiplier = 0.33;
        
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

    /**
     * Swerve class holds constants related to the swerve drive system.
     */
    public static final class Swerve {
        public static final int pigeonID = 1;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final Pose2d initialPose = new Pose2d(0, 0, new Rotation2d());

        // Drivetrain Constants
        public static final double trackWidth = Units.inchesToMeters(21.5); 
        public static final double wheelBase = Units.inchesToMeters(21.5);
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

        public static final frc.lib.motors.TalonFxConfiguration angleMotorConfig = new TalonFxConfiguration()
            .withContinuousCurrentLimit(25)
            .withPeakCurrentLimit(40)
            .withPeakCurrentDuration(0.1)
            .withPID(
                1.201173, // kP
                0.0, // kI
                0.024023, // kD
                0.0 // kF
            )
            .withoutNeutralModeBrake();

        public static final frc.lib.motors.TalonFxConfiguration driveMotorConfig = new TalonFxConfiguration()
            .withContinuousCurrentLimit(35)
            .withPeakCurrentLimit(60)
            .withPeakCurrentDuration(0.1)
            .withPID(
                0.20019, // kP
                0.0, // kI
                0.0, // kD
                0.0 // kF
            )
            .withNeutralModeBrake()
            .withOpenLoopRamp(0.5)
            .withClosedLoopRamp(0.5);

        // Drive Motor Characterization Values
        public static final double driveKS = (0.667 / 12); // divide by 12 to convert from volts to percent
                                                           // output for
                                                           // CTRE
        public static final double driveKV = (2.44 / 12);
        public static final double driveKA = (0.27 / 12);

        // Swerve Profiling Values
        public static final double maxSpeed = 1; // meters per second
        public static final double maxAngularVelocity = Math.PI * 1.5; // radians per second (rad/s)
        
        // Turbo mode values
        public static final double turboMaxSpeed = 3;
        public static final double turboMaxAngularVelocity = Math.PI * 1.5;

        // Module Specific Constants
        // Front Left Module - Module 0
        public static final class Mod0 {
            public static final int driveMotorID = 10;
            public static final int angleMotorID = 6;
            public static final int angleEncoderID = 2;

            public static final double angleOffset = 0.244873 * 360;

        }

        // Front Right Module - Module 1
        public static final class Mod1 {
            public static final int driveMotorID = 11;
            public static final int angleMotorID = 7;
            public static final int angleEncoderID = 3;

            public static final double angleOffset = 0.300781 * 360;

        }

        // Back Right Module - Module 2
        public static final class Mod2 {
            public static final int driveMotorID = 12;
            public static final int angleMotorID = 8;
            public static final int angleEncoderID = 4;

            public static final double angleOffset = 0.210693 * 360;
        }

        // Back Left Module - Module 3
        public static final class Mod3 {
            public static final int driveMotorID = 13;
            public static final int angleMotorID = 9;
            public static final int angleEncoderID = 5;

            public static final double angleOffset = 0.008057 * 360;
        }

          // field pose for shooting to speaker
        public static final FieldPose speakerPose = FieldPose.fromNative(new Pose3d(
            new Translation3d(
                Units.inchesToMeters(9.055), // x
                Units.inchesToMeters(218.42), // y
                Units.inchesToMeters(82.9)), // z
                new Rotation3d()
        ));

        // Autonomous 
        public static final HolonomicPathFollowerConfig pathFollowerConfig = 
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                new PIDConstants(0.220019, 0.0, 0.0), // Translation PID constants
                new PIDConstants(1.201173, 0.0, 0.024023), // Rotation PID constants
                4.5, // Max module speed, in m/s
                0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                new ReplanningConfig() // Default path replanning config. See the API for the options here
            );

        // Robot rotation locking

        public static final double rotationLockKP = 1.0;
        public static final double rotationLockKI = 0.0;
        public static final double rotationLockKD = 0.0;
    }


}
