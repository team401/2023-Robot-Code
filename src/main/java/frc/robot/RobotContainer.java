// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.commands.pivot.MovePivot;

public class RobotContainer {
    
    private final Drive drive = new Drive();
    private final PivotSubsystem pivot = new PivotSubsystem();
    private final TelescopeSubsystem telescope = new TelescopeSubsystem();
    private final WristSubsystem wrist = new WristSubsystem();

    private final Joystick leftStick = new Joystick(0);
    private final Joystick rightStick = new Joystick(1);
    private final XboxController gamepad = new XboxController(2);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        configureSubsystems();
        configureBindings();
        configureAutos();

    }

    private void configureSubsystems() {

        drive.setDefaultCommand(new DriveWithJoysticks(
            drive,
            () -> -leftStick.getRawAxis(1),
            () -> -leftStick.getRawAxis(0),
            () -> -rightStick.getRawAxis(0),
            true
        ));

    }

    private void configureBindings() {
        new Trigger(() -> rightStick.getRawButtonPressed(2))
            .onTrue(new InstantCommand(() -> drive.resetHeading()));

        new JoystickButton(leftStick, 1)
            .onTrue(new InstantCommand(() -> drive.setBabyMode(true)))
            .onFalse(new InstantCommand(() -> drive.setBabyMode(false)));

        new JoystickButton(gamepad, Button.kB.value)
            .onTrue(new MovePivot(pivot, telescope, Math.PI / 4));

        new JoystickButton(gamepad, Button.kA.value)
            .onTrue(new MovePivot(pivot, telescope, Math.PI / 2));

        new JoystickButton(gamepad, Button.kRightBumper.value)
            .onTrue(new InstantCommand(() -> pivot.setVolts(4)))
            .onFalse(new InstantCommand(() -> pivot.setVolts(0)));
            
        new JoystickButton(gamepad, Button.kLeftBumper.value)
            .onTrue(new InstantCommand(() -> pivot.setVolts(-4)))
            .onFalse(new InstantCommand(() -> pivot.setVolts(0)));


    }

    private void configureAutos() {

    }

    public Command getAutonomousCommand() {
        return null;
    }   
}
