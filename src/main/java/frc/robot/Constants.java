// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;

public final class Constants {
    public static final class CANDevices {

        public static final String canivoreName = "Canivore";

        public static final int frontLeftDriveMotorID = 2;
        public static final int frontLeftRotationMotorID = 1;

        public static final int frontRightDriveMotorID = 4;
        public static final int frontRightRotationMotorID = 3;

        public static final int backLeftDriveMotorID = 8;
        public static final int backLeftRotationMotorID = 7;

        public static final int backRightDriveMotorID = 6;
        public static final int backRightRotationMotorID = 5;

        public static final int frontLeftRotationEncoderID = 9;
        public static final int frontRightRotationEncoderID = 10;
        public static final int backLeftRotationEncoderID = 12;
        public static final int backRightRotationEncoderID = 11;

        public static final int pigeonIMU = 20;

        public static final int leftPivotMotorID = 14;
        public static final int rightPivotMotorID = 13;
        
        public static final int pivotEncoderID = 0;

        public static final int telescopeMotorID = 18;

        public static final int wristMotorID = 17;

        public static final int rightIntakeMotorID = 15;
        public static final int leftIntakeMotorID = 16;

    }
    
    public static final class DriveConstants {

        public static final double trackWidth = Units.inchesToMeters(17.75); // distance between the left and right wheels
        public static final double wheelBase = Units.inchesToMeters(23.75); // distance between the front and rear wheels
        public static final double wheelRadiusM = Units.inchesToMeters(2.02125);

        public static final double driveWheelGearReduction = 6.86;
        public static final double rotationWheelGearReduction = 12.8;

        public static final double frontLeftAngleOffset = 2.93-3.14;
        public static final double frontRightAngleOffset = 1.33-3.14;
        public static final double backLeftAngleOffset = 1.68-3.14;
        public static final double backRightAngleOffset = 1.06;

        public static final double driveKp = 0.2;
        public static final double driveKd = 2.0;
        public static final double rotationKp = 4.7;
        public static final double rotationKd = 0.1;

        public static final double driveSnapKp = 1.5;
        public static final double driveSnapKi = 0;
        public static final double driveSnapKd = 0;

        public static final SwerveDriveKinematics kinematics = 
            new SwerveDriveKinematics(
                new Translation2d(trackWidth / 2.0, wheelBase / 2.0), //front left
                new Translation2d(trackWidth / 2.0, -wheelBase / 2.0), //front right
                new Translation2d(-trackWidth / 2.0, wheelBase / 2.0), //rear left
                new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0) //rear right
        );

        public static final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0.24, 3.265);

        public static final double maxDriveSpeed = 5;
        public static final double maxTurnRate = 2 * Math.PI;

        public static final double driveJoystickDeadbandPercent = 0.09;
        public static final double driveMaxJerk = 200.0;

    }

    public static final class PivotConstants {
        public static final double encoderOffsetRad = -Units.rotationsToRadians(0.1775);

        public static final double armToMotorGearRatio = 1 / 90;

        public static final double maxFwdRotationRad = Units.degreesToRadians(-30);
        public static final double maxBackRotationRad = Units.degreesToRadians(210);

        public static final double lengthWOTeleM = Units.inchesToMeters(27.75);

        public static final double kP = 14;
        public static final double kD = 0;
        public static final double kV = 1.18;
        public static final double kA = 0.03; // estimate
        public static final double kG = 0.25;
        public static final double kS = 0.18;
        public static final double extraKg = 0.62;
    }

    public static final class TelescopeConstants {
        public static final double maxPosM = 0.8;
        public static final double minPosM = 0.01;

        public static final double conversionM = 0.00865;

        public static final double kP = 60;
        public static final double kV = 4.7;
        public static final double kA = 0.09; // estimate
        public static final double kG = 0.3;
        public static final double kS = 0.25;
    }

    public static final class WristConstants {
        //TODO: set limits
        public static final double positiveLimitRad = 0.0;
        public static final double negativeLimitRad = 0.0;

        public static final double intakeLengthM = Units.inchesToMeters(10);

        public static final double gearRatio = 8.0 / 78;

        public static final double kP = 10;
        public static final double kI = 3;
        public static final double kV = 0.185;
        public static final double kA = 0.01; // estimate
        public static final double kG = 0.26;
        public static final double kS = 0.27;
    }

    public static final class ArmPositions {

        public static final double[] intakeConeBackShelf = new double[] {0.674, 0.64, -0.87};
        public static final double[] intakeCubeShelf = new double[] {0.59, 0.55, 0.15};
        public static final double[] intakeCubeGround = new double[] {-0.33, 0.086, 0.45};
        public static final double[] intakeConeBackGround = new double[] {-0.06, 0.059, -0.964};
        public static final double[] placeConeBackHigh = new double[] {0.558, 0.71, 1.5};
        public static final double[] placeConeBackMid = new double[] {0.551, 0.24, 1.02};
        public static final double[] placeCubeHigh = new double[] {0.550, 0.781, 1.5};
        public static final double[] placeCubeMid = new double[] {0.515, 0.276, 1.5};
        public static final double[] stow = new double[] {Math.PI / 2, 0.05, Math.PI / 2};

        public static final double wristConePlace = -0.88;
        public static final double[] wristConePlaceHigh = new double[] {0.558, 0.67, -0.9};

    }

    public static enum GamePieceMode {
        Cube,
        ConeUp,
        ConeForward,
        ConeBack
    }

    public static enum Position {
        Ground,
        Mid,
        High,
        Shelf,
        Stow
    }

    public static final class LEDConstants {

        // Ports
        public static final int ledPort = 0;

        // LED Data
        public static final int armLedCount = 123;
        public static final int baseLedCount = 128;

        // Rainbow
        public static final boolean dynamicRainbow = true;
        public static final int dynamicRainbowSpeed = 1;

        // Pre-Match Climb Pattern
        public static final int climbSpeed = 2;
        public static final int climbMaxDelay = 40;
        public static final int climbMinDelay = 20;
        public static final int climbMaxLength = 10;
        public static final int climbMinLength = 5;

        // Other
        public static final Color activeSideFlashColor = new Color(0, 0, 0);
        public static final Color intakeFlashColor = new Color(255, 255, 255);
        public static final Color whistleFlashColor = new Color(255, 179, 0);

    }

    public static final class VisionConstants {

        public static Transform3d vehicleToFrontCamera = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));
        public static Transform3d vehicleToBackCamera = new Transform3d(new Translation3d(0.2794, -0.1778, 0), new Rotation3d(0, 0, Math.PI));//0.23, 0.146

        public static final HashMap<Integer, Pose3d> tagMap = new HashMap<>() {{
            put(1, new Pose3d(new Translation3d(15.51, 1.07, 0.46), new Rotation3d(new Quaternion(0, 0, 0, 1))));
            put(2, new Pose3d(new Translation3d(15.51, 2.75, 0.46), new Rotation3d(new Quaternion(0, 0, 0, 1))));
            put(3, new Pose3d(new Translation3d(15.51, 4.42, 0.46), new Rotation3d(new Quaternion(0, 0, 0, 1))));
            put(4, new Pose3d(new Translation3d(16.18, 6.75, 0.70), new Rotation3d(new Quaternion(0, 0, 0, 1))));
            put(5, new Pose3d(new Translation3d(0.36, 6.75, 0.70), new Rotation3d(new Quaternion(1, 0, 0, 0))));
            put(6, new Pose3d(new Translation3d(1.03, 4.42, 0.46), new Rotation3d(new Quaternion(1, 0, 0, 0))));
            put(7, new Pose3d(new Translation3d(1.03, 2.75, 0.46), new Rotation3d(new Quaternion(1, 0, 0, 0))));
            put(8, new Pose3d(new Translation3d(1.03, 1.07, 0.46), new Rotation3d(new Quaternion(1, 0, 0, 0))));
        }};

    }

    public static final class AutoConstants {

        /*
        1-1: cone
        1-2: cone + balance
        2-1: nothing
        2-2: balance
        3-1: cone
        3-2: cone + balance
        */

        // hopefully we can increase these to 3 and 5
        public static final double kMaxVelocityMetersPerSecond = 2;//3
        public static final double kMaxAccelerationMetersPerSecondSquared = 2;

        public static final double autoTranslationKp = 2;
        public static final double autoTranslationKi = 0;
        public static final double autoTranslationKd = 0;

        public static final double autoRotationKp = 2;
        public static final double autoRotationKi = 0;
        public static final double autoRotationKd = 0;

        public static final double autoBalanceKp = 0.1;
        public static final double autoBalanceKi = 0.0;
        public static final double autoBalanceKd = 0.0;

        public static final double initialBalanceSpeed = 1;

    }

}