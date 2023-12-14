// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.generated.TunerConstants;

public class RobotContainer {
    CommandJoystick leftJoystick = new CommandJoystick(0);
    CommandJoystick rightJoystick = new CommandJoystick(1);
    CommandXboxController controller = new CommandXboxController(2);
    CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;

    Telemetry logger = new Telemetry(DriveConstants.MaxSpeedMetPerSec);

    private void configureBindings() {
        drivetrain.setDefaultCommand(new DriveWithJoysticks(drivetrain,
                () -> leftJoystick.getY(),
                () -> leftJoystick.getX(),
                () -> rightJoystick.getX(),
                () -> rightJoystick.trigger().getAsBoolean(),
                () -> leftJoystick.trigger().getAsBoolean()));

        rightJoystick.button(2)
                .whileTrue(new InstantCommand(() -> drivetrain.seedFieldRelative()));

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
