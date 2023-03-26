// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmPositions;
import frc.robot.Constants.GamePieceMode;
import frc.robot.Constants.Position;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.auto.AutoRoutines;
import frc.robot.commands.auto.Balance;
import frc.robot.commands.auto.DriveToPose;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDManager;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.PositionHelper;
import frc.robot.commands.pivot.HoldPivot;
import frc.robot.commands.pivot.MovePivot;
import frc.robot.commands.telescope.HoldTelescope;
import frc.robot.commands.telescope.HomeTelescope;
import frc.robot.commands.telescope.MoveTelescope;
import frc.robot.commands.wrist.HoldWrist;
import frc.robot.commands.wrist.HomeWrist;
import frc.robot.commands.wrist.MoveWrist;
import frc.robot.oi.XboxMasher;

public class RobotContainer {
    
    private final Drive drive = new Drive();
    private final PivotSubsystem pivot = new PivotSubsystem();
    private final TelescopeSubsystem telescope = new TelescopeSubsystem();
    private final WristSubsystem wrist = new WristSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final Vision vision = new Vision();
    private final LEDManager ledManager = new LEDManager();

    private final Joystick leftStick = new Joystick(0);
    private final Joystick rightStick = new Joystick(1);
    private final XboxMasher masher = new XboxMasher(new XboxController(2));

    private final XboxController gamepad = new XboxController(2);

    SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    private double volts = 2.5;

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
    }

    private void configureCompBindings() {
        
        // Drive
        new JoystickButton(rightStick, 1)
            .onTrue(new InstantCommand(() -> RobotState.getInstance().setBackOverride(true)))
            .onFalse(new InstantCommand(() -> RobotState.getInstance().setBackOverride(false)));
        
        new JoystickButton(leftStick, 2)
            .onTrue(new InstantCommand(() -> RobotState.getInstance().invertBack()));

        new JoystickButton(rightStick, 2)
            .onTrue(new InstantCommand(() -> drive.resetHeading()));

        new JoystickButton(leftStick, 1)
            .onTrue(new InstantCommand(() -> drive.setBabyMode(true)))
            .onFalse(new InstantCommand(() -> drive.setBabyMode(false)));

        new JoystickButton(leftStick, 2)
            .onTrue(new InstantCommand(() -> gamepad.setRumble(RumbleType.kBothRumble, 0.5)))
            .onFalse(new InstantCommand(() -> gamepad.setRumble(RumbleType.kBothRumble, 0)));

        // new JoystickButton(leftStick, 3)
        //     .whileTrue(new DriveToPose(drive, true));
        // new JoystickButton(leftStick, 4)
        //     .whileTrue(new DriveToPose(drive, false));

        // Overrides
        new JoystickButton(rightStick, 12)
            .onTrue(new HomeWrist(wrist));
        
        new JoystickButton(rightStick, 13)
            .onTrue(new HomeTelescope(telescope));
        
        new JoystickButton(rightStick, 15)
            .whileTrue(new RunCommand(() -> pivot.overrideVolts(3), pivot));

        new JoystickButton(rightStick, 14)
            .whileTrue(new RunCommand(() -> pivot.overrideVolts(-3), pivot));

        new JoystickButton(leftStick, 7)
            .onTrue(new InstantCommand(pivot::toggleKill, pivot));

        new JoystickButton(leftStick, 6)
            .onTrue(new InstantCommand(telescope::toggleKill, telescope));

        new JoystickButton(leftStick, 5)
            .onTrue(new InstantCommand(wrist::toggleKill, wrist));

        new JoystickButton(leftStick, 10)
            .onTrue(new InstantCommand(pivot::toggleKill))
            .onTrue(new InstantCommand(telescope::toggleKill))
            .onTrue(new InstantCommand(wrist::toggleKill));
             
        // Set game piece mode
        masher.cubeMode().onTrue(new InstantCommand(() ->
            RobotState.getInstance().setMode(GamePieceMode.Cube)));           
        masher.coneDownMode().onTrue(new InstantCommand(() ->
            RobotState.getInstance().setMode(GamePieceMode.ConeDown)));
        masher.coneUpMode().onTrue(new InstantCommand(() ->
            RobotState.getInstance().setMode(GamePieceMode.ConeUp)));
        
        // Move arm
        masher.high().onTrue(getMoveArmCommand(Position.High)); // move to high
        masher.mid().onTrue(getMoveArmCommand(Position.Mid)); // move to mid
        masher.ground().onTrue(getMoveArmCommand(Position.Ground)); // move to ground
        masher.stow().onTrue(getMoveArmCommand(Position.Stow)); // move to stow    
        masher.shelf().onTrue(getMoveArmCommand(Position.Shelf)); // move to shelf

        masher.special().onTrue(new MoveWrist(wrist, pivot, ArmPositions.wristConePlace));
        masher.otherSpecial().onTrue(new MoveWrist(wrist, pivot, ArmPositions.placeConeDownHigh[2]));
            
        // masher.flipSide().onTrue(
        //     new InstantCommand(() -> RobotState.getInstance().invertBack())); // flip side

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
            .onTrue(new InstantCommand(intake::toggleIntake)); // toggle intake

        masher.place()
            .onTrue(new InstantCommand(intake::place)) // start place
            .onFalse(new InstantCommand(intake::stop)); // stop place
        }

    private void configureAutos() {
        // Select path
        autoChooser.addOption("0-0", new AutoRoutines("0-0", drive, pivot, telescope, wrist, intake, vision));
        for (int start = 1; start <= 3; start++) {
            for (int path = 1; path <= 2; path++) {
                autoChooser.addOption("B-"+start+"-"+path, new AutoRoutines("B-"+start+"-"+path, drive, pivot, telescope, wrist, intake, vision));
                autoChooser.addOption("R-"+start+"-"+path, new AutoRoutines("R-"+start+"-"+path, drive, pivot, telescope, wrist, intake, vision));
            }
        }
        autoChooser.setDefaultOption("default", new AutoRoutines("B-1-1", drive, pivot, telescope, wrist, intake, vision));
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void enabledInit() {
        if (DriverStation.isTeleop()) {
            new HomeTelescope(telescope).schedule();
            new HomeWrist(wrist).schedule();
        }

        pivot.setDesiredSetpointRad(new State(Math.PI / 2, 0));
        wrist.updateDesiredSetpointRad(new State(Math.PI / 2, 0));
        telescope.setDesiredSetpoint(new State(0.1, 0));
    }

    public Command getMoveArmCommand(Position position) {
        return new InstantCommand(() -> {
            RobotState.getInstance().setStow(position.equals(Position.Stow));
            double[] positions = PositionHelper.getDouble(position, RobotState.getInstance().getMode());

            new MovePivot(pivot, positions[0]).schedule();
            new MoveTelescope(telescope, pivot, positions[1], positions[0]).schedule();
            new MoveWrist(wrist, pivot, positions[2]).schedule();
        });
    }
}