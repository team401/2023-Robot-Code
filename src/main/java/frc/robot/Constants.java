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
        
        public static final int frontLeftDriveMotorID = 0;
        public static final int frontLeftRotationMotorID = 1;

        public static final int frontRightDriveMotorID = 2;
        public static final int frontRightRotationMotorID = 3;

        public static final int backLeftDriveMotorID = 4;
        public static final int backLeftRotationMotorID = 5;

        public static final int backRightDriveMotorID = 6;
        public static final int backRightRotationMotorID = 7;

        public static final int frontLeftRotationEncoderID = 10;
        public static final int frontRightRotationEncoderID = 11;
        public static final int backLeftRotationEncoderID = 12;
        public static final int backRightRotationEncoderID = 13;

        public static final int pigeonIMU = 20;

    }
    
    public static final class DriveConstants {

        // TODO: Tune constants when we have new drivebase

        public static final double trackWidth = Units.inchesToMeters(19.75);
        public static final double wheelBase = Units.inchesToMeters(19.75);

        public static final double driveWheelGearReduction = 6.75;
        public static final double rotationWheelGearReduction = 150.0 / 7.0;
        public static final double maxSpeedMPerS = Units.feetToMeters(15.0);
        public static final double maxSpeedWhileShootingMPerS = Units.feetToMeters(10.0);
        public static final double maxAngularSpeedRadPerS = 2 * Math.PI;

        public static final double wheelRadiusM = Units.inchesToMeters(1.99868);

        public static final double frontLeftAngleOffset = 0.8605632220038447;
        public static final double frontRightAngleOffset = 5.750893973783269;
        public static final double backLeftAngleOffset = 1.2854759002481673;
        public static final double backRightAngleOffset = 4.275204455837282;

        public static final double rotationKp = 4.7;
        public static final double rotationKd = 0.1;
        public static final double driveKp = 0.2;
        public static final double driveKd = 2.0;

        public static final double followTrajectoryXControllerKp = 0.25;
        public static final double followTrajectoryXControllerKd = 0;

        public static final double followTrajectoryYControllerKp = 0.25;
        public static final double followTrajectoryYControllerKd = 0;

        public static final double followTrajectoryThetaControllerKp = 3.0;
        public static final double followTrajectoryThetaControllerKd = 0;

        public static final double driveJoystickDeadbandPercent = 0.075;
        public static final double driveMaxJerk = 200.0;

        public static final SwerveDriveKinematics kinematics = 
            new SwerveDriveKinematics(
                new Translation2d(trackWidth / 2.0, wheelBase / 2.0), //values for front left (+, +)
                new Translation2d(trackWidth / 2.0, -wheelBase / 2.0), //values for front right (+, -)
                new Translation2d(-trackWidth / 2.0, wheelBase / 2.0), //values for back left (-, +)
                new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0) //values for back right (-, -)
            );

        public static final SimpleMotorFeedforward driveModel = new SimpleMotorFeedforward(0.184, 0.1163414634);

    }

}