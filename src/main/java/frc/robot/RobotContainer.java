// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;

public class RobotContainer {
    final double MaxSpeedMetPerSec = 6;
    final double MaxAngularRateRadiansPerSec = Math.PI * 2; // 2 PI is one full rotation per second
    final double deadbandPercent = 0.16;

    /* Setting up bindings for necessary control of the swerve drive platform */
    // Add a keyboard joystick for testing
    CommandJoystick keyboardJoystick = new CommandJoystick(2); // Keyboard joystick for sim/debugging
    CommandJoystick leftJoystick = new CommandJoystick(0); // Left joystick
    CommandJoystick rightJoystick = new CommandJoystick(1); // Right joystick
    CommandXboxController controller = new CommandXboxController(2); // Button Masher
    CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
    SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withIsOpenLoop(true);
    SwerveRequest.RobotCentric driveRobot = new SwerveRequest.RobotCentric().withIsOpenLoop(true);
    SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    Telemetry logger = new Telemetry(MaxSpeedMetPerSec);

    // Grabs the left joystick x-axis with deadband applied
    @SuppressWarnings("unused")
    private double leftStickGetX() {
        double rawInput = leftJoystick.getX();
        return Math.abs(rawInput) > deadbandPercent ? rawInput : 0;
    }

    // Grabs the left joystick y-axis with deadband applied
    @SuppressWarnings("unused")
    private double leftStickGetY() {
        double rawInput = leftJoystick.getY();
        return Math.abs(rawInput) > deadbandPercent ? rawInput : 0;
    }

    // Grabs the right joystick x-axis with deadband applied
    @SuppressWarnings("unused")
    private double rightStickGetX() {
        double rawInput = rightJoystick.getX();
        return Math.abs(rawInput) > deadbandPercent ? rawInput : 0;
    }

    // Grabs the right joystick y-axis with deadband applied
    @SuppressWarnings("unused")
    private double rightStickGetY() {
        double rawInput = rightJoystick.getY();
        return Math.abs(rawInput) > deadbandPercent ? rawInput : 0;
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> drive.withVelocityX(-leftStickGetY() * MaxSpeedMetPerSec)
                        .withVelocityY(-leftStickGetX() * MaxSpeedMetPerSec)
                        .withRotationalRate(-rightStickGetX() * MaxAngularRateRadiansPerSec)));

        rightJoystick.trigger()
                .whileTrue(
                        drivetrain.applyRequest(() -> driveRobot.withVelocityX(-leftStickGetY() * MaxSpeedMetPerSec)
                                .withVelocityY(-leftStickGetX() * MaxSpeedMetPerSec)
                                .withRotationalRate(-rightStickGetX() * MaxAngularRateRadiansPerSec)));

        leftJoystick.trigger()
                .whileTrue(drivetrain
                        .applyRequest(() -> drive.withVelocityX(-leftStickGetY() * MaxSpeedMetPerSec * 0.5)
                                .withVelocityY(-leftStickGetX() * MaxSpeedMetPerSec * 0.5)
                                .withRotationalRate(-rightStickGetX() * MaxAngularRateRadiansPerSec * 0.5)));

        rightJoystick.button(2)
                .whileTrue(new InstantCommand(() -> drivetrain.seedFieldRelative()));

        controller.a().whileTrue(drivetrain.applyRequest(() -> brake));
        controller.b().whileTrue(drivetrain
                .applyRequest(
                        () -> point
                                .withModuleDirection(new Rotation2d(-controller.getLeftY(), -controller.getLeftX()))));

        controller.x().whileTrue(drivetrain.applyRequest(() -> drive.withIsOpenLoop(false)));

        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public void configureSubsystems() {
    }

    public RobotContainer() {
        configureBindings();
        configureSubsystems();
    }

    public void disabledPeriodic() {
    }

    public void enabledInit() {
    }
}
