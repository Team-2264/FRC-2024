// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.OptionalDouble;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.AngleEstimation;
import frc.lib.FieldPose;
import frc.lib.GravityTrajectoryParameters;
import frc.lib.LinearTrajectoryParameters;
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
        public static final int controller2Port = 1;
        public static final double stickDeadband = 0.1;

    }

    /**
     * Targeting class holds constants related to targeting.
     */
    public static final class Targeting {
          // field pose for shooting to speaker
        public static FieldPose blueSpeakerPose = FieldPose.fromWpiBlue(new Pose3d(
            new Translation3d(
                Units.inchesToMeters(0), // x
                Units.inchesToMeters(218.42), // y
                Units.inchesToMeters(82.9)), // z
                new Rotation3d()
        ));

        public static FieldPose redSpeakerPose = FieldPose.fromWpiBlue(new Pose3d(
            new Translation3d(
                Units.inchesToMeters(652.73), // x
                Units.inchesToMeters(218.42), // y
                Units.inchesToMeters(82.9)), // z
                new Rotation3d()
        ));

        public static double pivotToGround = 0.3001;
        public static double pivotToCenter = 0.2286;
        public static double logicalArmOffset = 12.742 / 180.0 * Math.PI;

        public static GravityTrajectoryParameters armParameters = new GravityTrajectoryParameters()
            .withArmLength(0.5917)
            .withGoalHeight(blueSpeakerPose.getWpiBlue().getZ() - pivotToGround)
            .withLaunchAngleOffset(107.258 * (Math.PI/180.0))
            .withLaunchVelocity(25);

        public static LinearTrajectoryParameters linearParameters = LinearTrajectoryParameters.fromGravityParameters(armParameters);

         /** 
         *  Get the arm angle to hit the speaker
         *  Visualiation: https://www.desmos.com/calculator/wgwhg7msod
         * 
         *  @param targetDistance The distance to the speaker
         *  @return The angle to hit the speaker in degrees
         * 
         */
        public static final OptionalDouble getSpeakerArmAngle(double targetDistance) {
            double pivotDistance = targetDistance - pivotToCenter;
            SmartDashboard.putNumber("Pivot distance", pivotDistance);

            AngleEstimation estimate;
            if(targetDistance > 2) {
                 estimate = armParameters.getEstimate(pivotDistance, 32);
            } else {
                 estimate = linearParameters.getEstimate(pivotDistance, 32);
            }

            SmartDashboard.putNumber("Raw angle estimate", Math.toDegrees(estimate.estimate));
            SmartDashboard.putNumber("Raw angle inaccurcy", estimate.inaccuracy);
    
            if(estimate.inaccuracy < 0.1) {
                return OptionalDouble.of(0.5 - (estimate.estimate + logicalArmOffset) * (1/(2*Math.PI)));
            } else {
                return OptionalDouble.empty();
            }
            
        }
        
        /**
         *  Gets a offset for the target height of the speaker based on the distance.
         * 
         * @param distance The distance to the speaker
         * @return The height offset in inchs
         */
        public static final double heightOffset(double distance) {
            return (Math.pow(((0.7 * distance) - 0.9), 3));

        }

         /**
         *  Gets the speed of the flywheel based on the distance to the speaker.
         * 
         * @param distance The distance to the speaker
         * @return The speed of the flywheel from 0 to 1 (percent output)
         */
        public static final double getFlywheelSpeed(double distance) {
            return Math.min(1.0, ((0.3 * distance) + 0.4));
        }

    }
    
    /**
     * Arm class holds constants related to the arm.
     */
    public static final class Arm {
        public static final NeoConfiguration[] shoulderNeoConfigs = new NeoConfiguration[] {
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
            0.025, // kG 
            0.0, // kV 
            0.0  // kA 
        );

        public static PIDController shoulderFeedback = new PIDController(
            10, // kP
            0.01, // kI
            0.1  // kD
        );

        public static final double shoulderMaxPower = 0.50;
        public static final double shoulderMaxAngle = 115.0 / 360.0;
        
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
            .withKD(0.0)
            .withBrakeMode(true);

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

        public static final double flywheelBaseVoltage = 10.0;

        public static final int[] beamBreakPorts = new int[]{2, 3};
        
        public static final double intakeSpeed = 0.6;

    }

    public static final class Climbing {
        public static final NeoConfiguration winchNeoConfig = new NeoConfiguration(27)
            .withKP(0.0)
            .withKI(0.0)
            .withKD(0.0)
            .withBrakeMode(true);
        
        public static final double accendSpeed = 1;
        public static final double descendSpeed = 1;

    }

    /**
     * Vision class holds constants related to the vision system.
     */
    public static final class Vision {
        public static final String cameraName = "apriltag";

        public static final Matrix<N3, N1> visionStandardDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30));
        public static double singleTargetMultiplier = 0;
        
        // Robot to Camera Transform
        public static final Transform3d robotToCamera = new Transform3d(
            new Translation3d(
                -Units.inchesToMeters(12), 
                Units.inchesToMeters(0), 
                Units.inchesToMeters(14)
            ),
            new Rotation3d(0, 0, Math.PI)
            
        );

    }

    /**
     * LedStrip class holds constants related to the LED strip.
     */
    public static final class LedStrip {
        public static final int pwmPort = 0;
        public static final int numLeds = 8;
        public static final float scaleFactor = 0.5f;
    }

    /**
     * Swerve class holds constants related to the swerve drive system.
     */
    public static final class Swerve {
        public static final int pigeonID = 1;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final Pose2d initialPose = new Pose2d(
            Units.inchesToMeters(39.3701 * 2), // x
            Units.inchesToMeters(218.42), // y
            new Rotation2d()
        );
     
        // Drivetrain Constants
        public static final double trackWidth = Units.inchesToMeters(22); // 21.5 previously
        public static final double wheelBase = Units.inchesToMeters(22); // 21.5 previously
        public static final double wheelDiameter = Units.inchesToMeters(4.05);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double openLoopRamp = 0.5;
        public static final double closedLoopRamp = 0.5;

        public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1
        public static final double angleGearRatio = (12.8 / 1.0); // 12.8:1

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0)
            
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
        public static final double driveKS = (0.667 / 12); // divide by 12 to convert from volts to percent output for CTRE
        public static final double driveKV = (2.44 / 12);
        public static final double driveKA = (0.27 / 12);

        // Swerve Profiling Values
        public static final double maxSpeed = 4.785; // meters per second
        public static final double maxAngularVelocity = Math.PI; // radians per second (rad/s)
        
        // Turbo mode values
        public static final double turboMaxSpeed = 4.785;
        public static final double turboMaxAngularVelocity = Math.PI;

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

        // Autonomous 
        public static final HolonomicPathFollowerConfig pathFollowerConfig = 
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                new PIDConstants(0.220019, 0.0, 0.0), // Translation PID constants
                new PIDConstants(1.201173, 0.0, 0.024023), // Rotation PID constants
                10, // Max module speed, in m/s - previously 4.5
                0.395, // Drive base radius in meters. Distance from robot center to furthest module. - previously 0.4
                new ReplanningConfig() // Default path replanning config. See the API for the options here
            );

        // Robot rotation locking
        public static final double rotationLockKP = 2.0;
        public static final double rotationLockKI = 0.01;
        public static final double rotationLockKD = 0;

    }

}
