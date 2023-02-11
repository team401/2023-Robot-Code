// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmPositions;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.commands.pivot.HoldPivot;
import frc.robot.commands.pivot.MovePivot;
import frc.robot.commands.telescope.HoldTelescope;
import frc.robot.commands.telescope.MoveTelescope;

public class RobotContainer {
    
    private final Drive drive = new Drive();
    private final PivotSubsystem pivot = new PivotSubsystem();
    private final TelescopeSubsystem telescope = new TelescopeSubsystem();
    // private final WristSubsystem wrist = new WristSubsystem();

    private final Joystick leftStick = new Joystick(0);
    private final Joystick rightStick = new Joystick(1);
    private final XboxController gamepad = new XboxController(2);

    private boolean rumbling;
    private final Command endRumbleCommand = new InstantCommand(() -> {
		gamepad.setRumble(RumbleType.kBothRumble, 0);
		rumbling = false;
	});

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

        pivot.setDefaultCommand(new HoldPivot(pivot, telescope));
        telescope.setDefaultCommand(new HoldTelescope(telescope, pivot));

    }

    private void configureBindings() {
        new Trigger(() -> rightStick.getRawButtonPressed(2))
            .onTrue(new InstantCommand(() -> drive.resetHeading()));

        new JoystickButton(leftStick, 1)
            .onTrue(new InstantCommand(() -> drive.setBabyMode(true)))
            .onFalse(new InstantCommand(() -> drive.setBabyMode(false)));


        new POVButton(gamepad, 90)
            .onTrue(new MovePivot(pivot, telescope, ArmPositions.mid[0]))
            .onTrue(new MoveTelescope(telescope, pivot, ArmPositions.mid[1],
                ArmPositions.mid[0]))
            .onTrue(endRumbleCommand);
        
        new POVButton(gamepad, 0)
            .onTrue(new MovePivot(pivot, telescope, ArmPositions.high[0]))
            .onTrue(new MoveTelescope(telescope, pivot, ArmPositions.high[1],
                ArmPositions.high[0]))
            .onTrue(endRumbleCommand);

        new POVButton(gamepad, 180)
            .onTrue(new MovePivot(pivot, telescope, ArmPositions.low[0]))
            .onTrue(new MoveTelescope(telescope, pivot, ArmPositions.low[1],
                ArmPositions.low[0]))
            .onTrue(endRumbleCommand);

        new JoystickButton(gamepad, Button.kY.value)
			.onTrue(new InstantCommand(() -> RobotState.getInstance().invertBack()))
			.onTrue(new InstantCommand(() -> {
				if (rumbling) {
					gamepad.setRumble(RumbleType.kBothRumble, 0);
				} else {
					gamepad.setRumble(RumbleType.kBothRumble, 1);
				}
				rumbling = !rumbling;
			}));

        new JoystickButton(gamepad, Button.kA.value)
            .onTrue(new InstantCommand(() -> telescope.setSimPos(0.2)));

        new JoystickButton(gamepad, Button.kB.value)
            .onTrue(new InstantCommand(() -> telescope.setSimPos(0.4)));


    }

    private void configureAutos() {

    }

    public Command getAutonomousCommand() {
        return null;
    }   
}
