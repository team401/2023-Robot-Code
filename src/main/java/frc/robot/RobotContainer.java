// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmPositions;
import frc.robot.Constants.GamePieceMode;
import frc.robot.Constants.Position;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.commands.pivot.HoldPivot;
import frc.robot.commands.pivot.MovePivot;
import frc.robot.commands.pivot.TestMovePivot;
import frc.robot.commands.telescope.HoldTelescope;
import frc.robot.commands.telescope.HomeTelescope;
import frc.robot.commands.telescope.MoveTelescope;
import frc.robot.commands.telescope.TestMoveTelescope;
import frc.robot.commands.wrist.HoldWrist;
import frc.robot.commands.wrist.HomeWrist;
import frc.robot.commands.wrist.MoveWrist;
import frc.robot.commands.wrist.TestMoveWrist;

public class RobotContainer {
    
    private final Drive drive = new Drive();
    private final PivotSubsystem pivot = new PivotSubsystem();
    private final TelescopeSubsystem telescope = new TelescopeSubsystem();
    private final WristSubsystem wrist = new WristSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();

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
        // configureCompBindings();
        configureTestBindings();
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

        // pivot.setDefaultCommand(new HoldPivot(pivot, telescope));
        // telescope.setDefaultCommand(new HoldTelescope(telescope, pivot));
        // wrist.setDefaultCommand(new HoldWrist(wrist, pivot));

    }

    private void configureTestBindings() {
        // Drive
        new Trigger(() -> rightStick.getRawButtonPressed(2))
            .onTrue(new InstantCommand(() -> drive.resetHeading()));

        new JoystickButton(leftStick, 1)
            .onTrue(new InstantCommand(() -> drive.setBabyMode(true)))
            .onFalse(new InstantCommand(() -> drive.setBabyMode(false)));

        // new JoystickButton(gamepad, Button.kB.value)
        //     .onTrue(new InstantCommand(() -> pivot.overrideVolts(1)))
        //     .onFalse(new InstantCommand(pivot::stop));

        // new JoystickButton(gamepad, Button.kX.value)
        //     .onTrue(new TestMovePivot(pivot, telescope));

        // new JoystickButton(gamepad, Button.kA.value)
        //     .onTrue(new TestMoveWrist(wrist, pivot));
        
        // new JoystickButton(gamepad, Button.kB.value)
            // .onTrue(new TestMoveWrist(wrist, pivot))
            // .onTrue(new TestMoveTelescope(telescope));

        new JoystickButton(gamepad, Button.kB.value)
            .onTrue(new InstantCommand(intake::runBackward))
            .onFalse(new InstantCommand(intake::stopMotor));

        new JoystickButton(gamepad, Button.kX.value)
            .onTrue(new InstantCommand(intake::runForward))
            .onFalse(new InstantCommand(intake::stopMotor));
    }

    private void configureCompBindings() {
        
        // Drive
        new Trigger(() -> rightStick.getRawButtonPressed(2))
            .onTrue(new InstantCommand(() -> drive.resetHeading()));

        new JoystickButton(leftStick, 1)
            .onTrue(new InstantCommand(() -> drive.setBabyMode(true)))
            .onFalse(new InstantCommand(() -> drive.setBabyMode(false)));
        
        // Set game piece mode
        new JoystickButton(gamepad, Button.kLeftBumper.value)
            .onTrue(new InstantCommand(() -> 
                RobotState.getInstance().setMode(GamePieceMode.ConeUp)));
            
        new JoystickButton(gamepad, Button.kRightBumper.value)
            .onTrue(new InstantCommand(()-> 
                RobotState.getInstance().setMode(GamePieceMode.Cube)));
        
        // Move arm
        new POVButton(gamepad, 0)
            .onTrue(getMoveCommand(Position.High))
            .onTrue(endRumbleCommand); // move to high
        new POVButton(gamepad, 90)
            .onTrue(getMoveCommand(Position.Mid))
            .onTrue(endRumbleCommand); // move to mid
        new POVButton(gamepad, 180)
            .onTrue(getMoveCommand(Position.Ground))
            .onTrue(endRumbleCommand); // move to ground
        new POVButton(gamepad, 270)
            .onTrue(getMoveCommand(Position.Stow))
            .onTrue(endRumbleCommand); // move to stow
            
        new JoystickButton(gamepad, Button.kX.value)
            .onTrue(getMoveCommand(Position.Shelf)); // move to shelf
            
        new JoystickButton(gamepad, Button.kY.value)
            .onTrue(new InstantCommand(() -> RobotState.getInstance().invertBack())) // flip
            .onTrue(new InstantCommand(() -> {
                if (rumbling) {
					gamepad.setRumble(RumbleType.kBothRumble, 0);
				} else {
					gamepad.setRumble(RumbleType.kBothRumble, 1);
				}
				rumbling = !rumbling;
            }));

        new JoystickButton(gamepad, Button.kB.value)
            .onTrue(pivot.killCommand())
            .onTrue(telescope.killCommand())
            .onTrue(wrist.killCommand()); // kill

        // Manually jog arm
        new Trigger(() -> gamepad.getLeftX() > 0.3)
            .onTrue(new InstantCommand(pivot::jogSetpointForward, pivot)); // jog pivot right
        new Trigger(() -> gamepad.getLeftX() < -0.3)
            .onTrue(new InstantCommand(pivot::jogSetpointBack, pivot)); // jog pivot left

        new Trigger(() -> gamepad.getRightX() > 0.3)
            .onTrue(new InstantCommand(telescope::jogSetpointForward, telescope)); // jog telescope up
        new Trigger(() -> gamepad.getRightX() < -0.3)
            .onTrue(new InstantCommand(telescope::jogSetpointBackward, telescope)); // jog telescope down

        new Trigger(() -> gamepad.getRightY() > 0.3)
            .onTrue(new InstantCommand(wrist::jogSetpointForward, wrist)); // jog wrist right
        new Trigger(() -> gamepad.getRightY() < -0.3)
            .onTrue(new InstantCommand(wrist::jogSetpointBack, wrist)); // jog wrist left

        // Intake
        new Trigger(() -> gamepad.getRightTriggerAxis() > 0.5)
            .onTrue(new InstantCommand(intake::runForward)) // start intake
            .onFalse(new InstantCommand(intake::stopMotor)); // stop intake

        new Trigger(() -> gamepad.getLeftTriggerAxis() > 0.5)
            .onTrue(new InstantCommand(intake::runBackward)) // start place
            .onFalse(new InstantCommand(intake::stopMotor)); // stop place
    }

    private void configureAutos() {

    }

    public Command getMoveCommand(Position position) {
        return new InstantCommand(() -> {
            double[] positions = PositionHelper.getDouble(
                position,
                RobotState.getInstance().getMode());
            
            new MovePivot(
                pivot,
                telescope,
                positions[0]).schedule();

            new MoveTelescope(
                telescope, 
                pivot, 
                positions).schedule();
                
            new MoveWrist(
                wrist, 
                pivot, 
                positions[2]).schedule();
        });
    }

    public Command getAutonomousCommand() {
        return null;
    }   

    public void enabledInit() {
        if (!telescope.homed) {
            //new HomeTelescope(telescope).schedule();
        }
        if (!wrist.homed) {
            //new HomeWrist(wrist).schedule();
        }
    }
}
