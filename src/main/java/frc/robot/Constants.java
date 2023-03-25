// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
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

        public static final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0.22, 2.151);

        public static final double maxDriveSpeed = 5;
        public static final double maxTurnRate = 2 * Math.PI;

        public static final double driveJoystickDeadbandPercent = 0.12;
        public static final double driveMaxJerk = 200.0;

        public static final double poseMoveTranslationkP = 3;
        public static final double poseMoveTranslationMaxVel = 4;
        public static final double poseMoveTranslationMaxAccel = 1;

        public static final double poseMoveRotationkP = 5;
        public static final double poseMoveRotationMaxVel = Math.PI;
        public static final double poseMoveRotationMaxAccel = Math.PI;

    }

    public static final class PivotConstants {
        public static final double encoderOffsetRad = -Units.rotationsToRadians(0.1775) + 0.01;

        public static final double armToMotorGearRatio = 1 / 90;

        public static final double maxFwdRotationRad = Units.degreesToRadians(-30);
        public static final double maxBackRotationRad = Units.degreesToRadians(210);

        public static final double lengthWOTeleM = Units.inchesToMeters(27.75);

        public static final double kP = 15;
        public static final double kD = 0.5;
        public static final double kV = 1.5;
        public static final double kA = 0.03; // estimate
        public static final double kG = 0.23;
        public static final double kS = 0.22;
        public static final double extraKg = 0.55;
    }

    public static final class TelescopeConstants {
        public static final double maxPosM = 0.8;
        public static final double minPosM = 0.01;

        public static final double conversionM = 0.00865;

        public static final double kP = 100;
        public static final double kV = 4.38;
        public static final double kA = 0.09; // estimate
        public static final double kG = 0.31; 
        public static final double kS = 0.42;
    }

    public static final class WristConstants {
        //TODO: set limits
        public static final double positiveLimitRad = Units.degreesToRadians(160);
        public static final double negativeLimitRad = Units.degreesToRadians(-160);

        public static final double intakeLengthM = Units.inchesToMeters(10);

        public static final double gearRatio = 8.0 / 78;

        public static final double kP = 10;
        public static final double kI = 3;
        public static final double kPCone = 40;
        public static final double kICone = 0;
        public static final double kPConeHold = 15;
        public static final double kIConeHold = 0;
        public static final double kV = 0.273;
        public static final double kA = 0.01; // estimate
        public static final double kG = 0.34;
        public static final double kS = 0.36;
        public static final double kVCone = 0.662;
        public static final double kACone = 0.01;
        public static final double kGCone = 0.55;
        public static final double kSCone = 0.45;
    }

    public static final class ArmPositions {
        public static final double[] intakeCubeShelf = new double[] {0.63, 0.55, 0.15};
        public static final double[] intakeCubeGround = new double[] {-0.31, 0.09, 0.25};
        public static final double[] intakeCubeExtendedGround = new double[] {-0.2, 0.65, 0.4};
        public static final double[] placeCubeHigh = new double[] {0.590, 0.781, 1.5};
        public static final double[] placeCubeMid = new double[] {0.555, 0.276, 1.5};

        public static final double[] intakeConeDownGround = new double[] {-0.02, 0.059, -1.1};
        public static final double[] intakeConeDownExtendedGround = new double[] {-0.02, 0.70, -1.1};
        public static final double[] intakeConeDownShelf = new double[] {0.714, 0.64, -0.87};
        public static final double[] placeConeDownHigh = new double[] {0.59, 0.71, 0.8};
        public static final double[] placeConeDownMid = new double[] {0.58, 0.22, 0.8};
        
        public static final double[] intakeConeUpGround = new double[] {0.05, 0.05, -0.24};
        public static final double[] intakeConeUpShelf = new double[] {0.714, 0.64, -0.7};
        public static final double[] placeConeUpHigh = new double[] {0.69, 0.78, -0.5};
        public static final double[] placeConeUpMid = new double[] {0.7, 0.21, -0.5};
        
        
        public static final double[] stow = new double[] {Math.PI / 2, 0.1, Math.PI / 2};
        
        public static final double wristConePlace = -0.88;
        public static final double[] placeConeDownHighAuto = new double[] {0.585, 0.70, 0.8};
        public static final double[] wristConePlaceHighAuto = new double[] {0.585, 0.70, -1.3};

    }

    public static enum GamePieceMode {
        Cube,
        ConeUp,
        ConeDown
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

        public static final String[] cameraNames = {
            "FrontLeft", 
            "FrontRight", 
            "BackLeft", 
            "BackRight"
        };

        public static final Transform3d[] vehicleToCameras = {//10 deg yaw, 5 deg pitch
            new Transform3d(new Translation3d(-0.03, 0.18, 0), new Rotation3d(0, Units.degreesToRadians(-5), Units.degreesToRadians(-10))),
            new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, Units.degreesToRadians(-5), Units.degreesToRadians(350))),
            new Transform3d(new Translation3d(0.03, 0.18, 0), new Rotation3d(0, Units.degreesToRadians(-5), Units.degreesToRadians(190))),
            new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, Units.degreesToRadians(-5), Units.degreesToRadians(190)))
        };

        public static final List<AprilTag> tags = new ArrayList<AprilTag>() {{
            add(new AprilTag(1, new Pose3d(new Translation3d(15.51, 1.08, 0.46), new Rotation3d(new Quaternion(0, 0, 0, 1)))));
            add(new AprilTag(2, new Pose3d(new Translation3d(15.51, 2.76, 0.46), new Rotation3d(new Quaternion(0, 0, 0, 1)))));
            add(new AprilTag(3, new Pose3d(new Translation3d(15.51, 4.52, 0.46), new Rotation3d(new Quaternion(0, 0, 0, 1)))));
            add(new AprilTag(4, new Pose3d(new Translation3d(16.18, 6.75, 0.70), new Rotation3d(new Quaternion(0, 0, 0, 1)))));
            add(new AprilTag(5, new Pose3d(new Translation3d(0.36, 6.75, 0.70), new Rotation3d(new Quaternion(1, 0, 0, 0)))));
            add(new AprilTag(6, new Pose3d(new Translation3d(1.03, 4.54, 0.46), new Rotation3d(new Quaternion(1, 0, 0, 0)))));
            add(new AprilTag(7, new Pose3d(new Translation3d(1.03, 2.77, 0.46), new Rotation3d(new Quaternion(1, 0, 0, 0)))));
            add(new AprilTag(8, new Pose3d(new Translation3d(1.03, 1.07, 0.46), new Rotation3d(new Quaternion(1, 0, 0, 0)))));
        }};

    }

    public static final class AutoConstants { // 16 ft recorded 5.25

        /*
            0-0: nothing
            1-1: 2.5
            1-2: 2 + balance
            2-1: 1
            2-2: 1 + balance
            3-1: 2.5
            3-2: 2 + balance
        */
        
        public static final double kMaxVelocityMetersPerSecond = 2;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;

        public static final double kMaxVelocitySlowMetersPerSecond = 2;
        public static final double kMaxAccelerationSlowMetersPerSecondSquared = 3;

        public static final double autoTranslationXKp = 10;
        public static final double autoTranslationXKi = 0;
        public static final double autoTranslationXKd = 0;
        public static final double autoTranslationYKp = 10;
        public static final double autoTranslationYKi = 0;
        public static final double autoTranslationYKd = 0;

        public static final double autoRotationKp = 4;
        public static final double autoRotationKi = 0.1;
        public static final double autoRotationKd = 0;

        public static final double autoBalanceKp = 0.3;
        public static final double autoBalanceKi = 0.05;
        public static final double autoBalanceKd = 0.0;

        public static final double initialBalanceSpeed = 1;

    }

}