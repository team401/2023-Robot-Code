// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

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
        public static final double wheelRadiusM = 0.050686;

        public static final double driveWheelGearReduction = 6.86;
        public static final double rotationWheelGearReduction = 12.8;

        public static final double frontLeftAngleOffset = 0.905;
        public static final double frontRightAngleOffset = 4.435;
        public static final double backLeftAngleOffset = 4.798;
        public static final double backRightAngleOffset = 1.055;

        public static final double driveKp = 0.2;
        public static final double driveKd = 2.0;
        public static final double rotationKp = 4.7;
        public static final double rotationKd = 0.1;


        public static final SwerveDriveKinematics kinematics = 
            new SwerveDriveKinematics(
                new Translation2d(trackWidth / 2.0, wheelBase / 2.0), //front left
                new Translation2d(trackWidth / 2.0, -wheelBase / 2.0), //front right
                new Translation2d(-trackWidth / 2.0, wheelBase / 2.0), //rear left
                new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0) //rear right
        );

        public static final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0.184, 0.1163414634);

        public static final double maxDriveSpeed = 14.4;
        public static final double maxTurnRate = 2 * Math.PI;

        public static final double driveJoystickDeadbandPercent = 0.075;
        public static final double driveMaxJerk = 200.0;

    }

    public static final class PivotConstants {
        //TODO: Set all

        public static final double encoderOffsetRad = -Units.rotationsToRadians(0.1775);

        public static final double armToMotorGearRatio = 1 / 90;

        public static final double maxFwdRotationRad = Units.degreesToRadians(-30);
        public static final double maxBackRotationRad = Units.degreesToRadians(210);

        public static final double kP = 18;
        public static final double kD = 0;
        public static final double kV = 1.18;
        public static final double kA = 0.03;
        public static final double kG = 0.25;
        public static final double kS = 0.18;
        public static final double extraKg = 0.62;
    }

    public static final class TelescopeConstants {
        public static final double maxPosM = 0.8; //TODO: Find Value
        public static final double minPosM = 0.05;

        // public static final double wheelRadiusM = Units.inchesToMeters(0.75); 
        // public static final double gearRatio = 16.0 / 60.0;
        public static final double conversionM = 0.00865;

        public static final double kP = 25;
        public static final double kV = 0.0;
        public static final double kA = 0.0;
        public static final double kG = 0.0;
        public static final double kS = 0.25;
    }

    public static final class WristConstants {
        //TODO: set all values
        public static final double positiveLimitRad = 0.0;
        public static final double negativeLimitRad = 0.0;

        public static final double gearRatio = 78.0 / 8;

        public static final double kP = 7;
        public static final double kI = 5;
        public static final double kV = 0.16; // estimate
        public static final double kA = 0.01; // estimate
        public static final double kG = 0.6;
        public static final double kS = 0.0;
    }

    public static final class ArmPositions {
        public static final double[] testMid = new double[] 
            {Units.degreesToRadians(15),
            0.5,
            Units.degreesToRadians(0)};
            
        public static final double[] testLow = new double[] 
            {Units.degreesToRadians(-10),
            0.2,
            Units.degreesToRadians(0)};

        public static final double[] testHigh = new double[] 
            {Units.degreesToRadians(80),
            0.7,
            Units.degreesToRadians(0)};

        public static final double[] intakeConeUpShelf = new double[] {};
        public static final double[] intakeCubeShelf = new double[] {};
        public static final double[] intakeCubeGround = new double[] {};
        public static final double[] intakeConeUpGround = new double[] {};
        public static final double[] placeConeUpHigh = new double[] {};
        public static final double[] placeConeUpMid = new double[] {};
        public static final double[] placeCubeHigh = new double[] {};
        public static final double[] placeCubeMid = new double[] {};
        public static final double[] stow = new double[] {0.39, 0.05, 0.0};

    }

    public static enum GamePieceMode {
        Cube,
        ConeUp,
        ConeForward,
        ConeBackward
    }

    public static enum Position {
        Ground,
        Mid,
        High,
        Shelf,
        Stow
    }


}