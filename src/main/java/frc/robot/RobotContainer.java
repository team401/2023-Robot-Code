// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmPositions;
import frc.robot.Constants.GamePieceMode;
import frc.robot.Constants.Position;
import frc.robot.Constants.TelescopeConstants;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDManager;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.commands.pivot.CharacterizePivot;
import frc.robot.commands.pivot.HoldPivot;
import frc.robot.commands.pivot.MovePivot;
import frc.robot.commands.pivot.TestMovePivot;
import frc.robot.commands.telescope.CharacterizeTelescope;
import frc.robot.commands.telescope.HoldTelescope;
import frc.robot.commands.telescope.HomeTelescope;
import frc.robot.commands.telescope.MoveTelescope;
import frc.robot.commands.telescope.TestMoveTelescope;
import frc.robot.commands.wrist.CharacterizeWrist;
import frc.robot.commands.wrist.HoldWrist;
import frc.robot.commands.wrist.HomeWrist;
import frc.robot.commands.wrist.MoveWrist;
import frc.robot.commands.wrist.TestMoveWrist;
import frc.robot.oi.ButtonMasher;
import frc.robot.oi.XboxMasher;

public class RobotContainer {
    
    private final Drive drive = new Drive();
    private final PivotSubsystem pivot = new PivotSubsystem();
    private final TelescopeSubsystem telescope = new TelescopeSubsystem();
    private final WristSubsystem wrist = new WristSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final LEDManager ledManager = new LEDManager();

    private final Joystick leftStick = new Joystick(0);
    private final Joystick rightStick = new Joystick(1);
    private final ButtonMasher masher = new XboxMasher(new XboxController(2));

    private final XboxController gamepad = new XboxController(2);

    private boolean rumbling;
    private final Command endRumbleCommand = new InstantCommand(() -> {
		gamepad.setRumble(RumbleType.kBothRumble, 0);
		rumbling = false;
	});

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        configureSubsystems();
        configureCompBindings();
        // configureTestBindings();
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
        wrist.setDefaultCommand(new HoldWrist(wrist, pivot));

    }

    private void configureTestBindings() {
        // Drive
        new Trigger(() -> rightStick.getRawButtonPressed(2))
            .onTrue(new InstantCommand(() -> drive.resetHeading()));

        new JoystickButton(leftStick, 1)
            .onTrue(new InstantCommand(() -> drive.setBabyMode(true)))
            .onFalse(new InstantCommand(() -> drive.setBabyMode(false)));

            
        new POVButton(gamepad, 270)
            .onTrue(new MovePivot(pivot, telescope, () -> ArmPositions.stow[0]))
            .onTrue(new MoveTelescope(telescope, pivot, () -> ArmPositions.stow[1], () -> ArmPositions.stow[0]))
            .onTrue(new MoveWrist(wrist, pivot, () -> ArmPositions.stow[2]));

        new POVButton(gamepad, 90)
            .onTrue(new MovePivot(pivot, telescope, () -> ArmPositions.placeConeBackHigh[0]))
            .onTrue(new MoveTelescope(telescope, pivot, () -> ArmPositions.placeConeBackHigh[1], () -> ArmPositions.placeConeBackHigh[0]))
            .onTrue(new MoveWrist(wrist, pivot, () -> ArmPositions.placeConeBackHigh[2]));
        
        new POVButton(gamepad, 180)
            .onTrue(new MovePivot(pivot, telescope, () -> ArmPositions.intakeConeBackShelf[0]))
            .onTrue(new MoveTelescope(telescope, pivot, () -> ArmPositions.intakeConeBackShelf[1], () -> ArmPositions.intakeConeBackShelf[0]))
            .onTrue(new MoveWrist(wrist, pivot, () -> ArmPositions.intakeConeBackShelf[2]));

        new JoystickButton(gamepad, Button.kA.value)
            .onTrue(new MoveWrist(wrist, pivot, () -> ArmPositions.wristConePlace));

        // new JoystickButton(gamepad, Button.kX.value)
        //     .onTrue(new InstantCommand(intake::runForward))
        //     .onFalse(new InstantCommand(intake::stopMotor));

        // new JoystickButton(gamepad, Button.kB.value)
        //     .onTrue(new InstantCommand(intake::runBackward))
        //     .onFalse(new InstantCommand(intake::stopMotor));

        // new JoystickButton(gamepad, Button.kY.value)
        //     .onTrue(new InstantCommand(intake::placeCube))
        //     .onFalse(new InstantCommand(intake::stopMotor));        

        // new JoystickButton(gamepad, Button.kB.value)
        //     .onTrue(new MovePivot(pivot, telescope, () -> 0.2))
        //     .onTrue(new MoveTelescope(telescope, pivot, () -> 0.06, () -> 0.2))
        //     .onFalse(new InstantCommand(() -> pivot.stop(), pivot));

        // new JoystickButton(gamepad, Button.kX.value)
        //     .onTrue(new MovePivot(pivot, telescope, () -> 3))
        //     .onTrue(new MoveTelescope(telescope, pivot, () -> 0.5, () -> 3))
        //     .onFalse(new InstantCommand(() -> pivot.stop(), pivot));
    }

    private void configureCompBindings() {
        
        // Drive
        new JoystickButton(rightStick, 2)
            .onTrue(new InstantCommand(() -> drive.resetHeading()));

        new JoystickButton(leftStick, 1)
            .onTrue(new InstantCommand(() -> drive.setBabyMode(true)))
            .onFalse(new InstantCommand(() -> drive.setBabyMode(false)));

        new JoystickButton(leftStick, 2)
            .onTrue(new InstantCommand(wrist::zeroOffset));
        
        // Set game piece mode
        masher.cubeMode().onTrue(new InstantCommand(() ->
            RobotState.getInstance().setMode(GamePieceMode.Cube)));           
        masher.coneMode().onTrue(new InstantCommand(() ->
            RobotState.getInstance().setMode(GamePieceMode.ConeBack)));
        
        // Move arm
        masher.high().onTrue(getMoveCommand(Position.High)); // move to high
        masher.mid().onTrue(getMoveCommand(Position.Mid)); // move to mid
        masher.ground().onTrue(getMoveCommand(Position.Ground)); // move to ground
        masher.stow().onTrue(getMoveCommand(Position.Stow)); // move to stow    
        masher.shelf().onTrue(getMoveCommand(Position.Shelf)); // move to shelf

        masher.special().onTrue(new MoveWrist(wrist, pivot, () -> ArmPositions.wristConePlace));
            
        masher.flipSide().onTrue(
            new InstantCommand(() -> RobotState.getInstance().invertBack())); // flip side

        masher.killAll()
            .onTrue(pivot.killCommand())
            .onTrue(wrist.killCommand())
            .onTrue(telescope.killCommand()); // kill

        // Manually jog arm
        masher.jogPivotUp()
            .onTrue(new InstantCommand(pivot::jogSetpointForward, pivot)); // jog pivot right
        masher.jogPivotDown()
            .onTrue(new InstantCommand(pivot::jogSetpointBack, pivot)); // jog pivot left

        masher.jogTelescopeUp()
            .onTrue(new InstantCommand(telescope::jogSetpointForward, telescope)); // jog telescope up
        masher.jogTelescopeDown()
            .onTrue(new InstantCommand(telescope::jogSetpointBackward, telescope)); // jog telescope down

        masher.jogWristUp()
            .onTrue(new InstantCommand(wrist::jogSetpointForward, wrist)); // jog wrist right
        masher.jogWristDown()
            .onTrue(new InstantCommand(wrist::jogSetpointBack, wrist)); // jog wrist left

        // Intake
        masher.intake()
            .onTrue(new InstantCommand(intake::intake)) // start intake
            .onFalse(new InstantCommand(intake::stopMotor)); // stop intake

        masher.place()
            .onTrue(new InstantCommand(intake::place)) // start place
            .onFalse(new InstantCommand(intake::stopMotor)); // stop place
        }

    private void configureAutos() {

    }

    public Command getAutonomousCommand() {
        return null;
    }

    public Command getMoveCommand(Position position) {

        RobotState.getInstance().setStow(position.equals(Position.Stow));
        
        return new InstantCommand(() -> {
            double[] positions = PositionHelper.getDouble(
                position,
                RobotState.getInstance().getMode());
            
            new MovePivot(
                pivot,
                telescope,
                () -> positions[0]).schedule();

            new MoveTelescope(
                telescope, 
                pivot, 
                () -> positions[1],
                () -> positions[0]).schedule();
                
            new MoveWrist(
                wrist, 
                pivot, 
                () -> positions[2]).schedule();
        });
    }

    public void enabledInit() {
        if (!telescope.homed) {
            new HomeTelescope(telescope).schedule();
        }
        if (!wrist.homed) {
            new HomeWrist(wrist).schedule();
        }

        pivot.setDesiredSetpointRad(new TrapezoidProfile.State(pivot.getPositionRad(), 0));
    }
}
