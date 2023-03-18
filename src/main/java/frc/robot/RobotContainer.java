// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ArmPositions;
import frc.robot.Constants.GamePieceMode;
import frc.robot.Constants.Position;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.DriveWithJoysticksSnap;
import frc.robot.commands.auto.AutoRoutines;
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
import frc.robot.oi.ButtonMasher;
import frc.robot.oi.XboxMasher;

public class RobotContainer {
    
    private final Drive drive = new Drive();
    private final PivotSubsystem pivot = new PivotSubsystem();
    private final TelescopeSubsystem telescope = new TelescopeSubsystem();
    private final WristSubsystem wrist = new WristSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final Vision vision = new Vision();
    private final LEDManager ledManager = new LEDManager();
    // private final AutoManager autoManager = new AutoManager(pivot, telescope, wrist, drive, intake, (position) -> getMoveCommand(position));

    private final Joystick leftStick = new Joystick(0);
    private final Joystick rightStick = new Joystick(1);
    private final XboxMasher masher = new XboxMasher(new XboxController(2));

    private final XboxController gamepad = new XboxController(2);

    SendableChooser<Command> autoChooser = new SendableChooser<Command>();

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

        // new JoystickButton(gamepad, Button.kA.value)
        //     .whileTrue(new RepeatCommand(new InstantCommand(() -> {
        //         armPositionTheta = gamepad.getRightTriggerAxis();
        //         double[] pos = ArmPositionCalc.findPositions(armPositionX, armPositionY, armPositionTheta);
        //         new ParallelCommandGroup(
        //             new MovePivot(pivot, telescope, () -> pos[0]),
        //             new MoveTelescope(telescope, pivot, () -> pos[1], () -> pos[1]),
        //             new MoveWrist(wrist, pivot, () -> pos[2])
        //         ).schedule();
        //     })));
        
        // new POVButton(gamepad, 0)
        //     .onTrue(new InstantCommand(() -> {
        //         armPositionY += 0.1;
        //     }));

        // new POVButton(gamepad, 180)
        //     .onTrue(new InstantCommand(() -> {
        //         armPositionY -= 0.1;
        //     }));
        
        // new POVButton(gamepad, 90)
        //     .onTrue(new InstantCommand(() -> {
        //         armPositionX += 0.1;
        //     }));
        
        // new POVButton(gamepad, 270)
        //     .onTrue(new InstantCommand(() -> {
        //         armPositionX -= 0.1;
        //     }));
    }

    private void configureCompBindings() {
        
        // Drive
        new JoystickButton(rightStick, 1)
            .onTrue(new InstantCommand(() -> RobotState.getInstance().invertBack()));
            // .whileTrue(
            //     new DriveWithJoysticksSnap(
            //         drive,
            //         () -> -leftStick.getRawAxis(1),
            //         () -> -leftStick.getRawAxis(0),
            //         true
            //     )
            // );

        new JoystickButton(rightStick, 2)
            .onTrue(new InstantCommand(() -> drive.resetHeading()));

        new JoystickButton(leftStick, 1)
            .onTrue(new InstantCommand(() -> drive.setBabyMode(true)))
            .onFalse(new InstantCommand(() -> drive.setBabyMode(false)));

        // Overrides
        new JoystickButton(rightStick, 12)
            .onTrue(new HomeWrist(wrist));
        
        new JoystickButton(rightStick, 13)
            .onTrue(new HomeTelescope(telescope));
        
        new JoystickButton(rightStick, 15)
            .whileTrue(new RunCommand(() -> pivot.overrideVolts(1.5), pivot));

        new JoystickButton(rightStick, 14)
            .whileTrue(new RunCommand(() -> pivot.overrideVolts(-1.5), pivot));

        new JoystickButton(leftStick, 7)
            .onTrue(new InstantCommand(pivot::toggleKill));

        new JoystickButton(leftStick, 6)
            .onTrue(new InstantCommand(telescope::toggleKill));

        new JoystickButton(leftStick, 5)
            .onTrue(new InstantCommand(wrist::toggleKill));

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
        masher.high().onTrue(getMoveCommand(Position.High)); // move to high
        masher.mid().onTrue(getMoveCommand(Position.Mid)); // move to mid
        masher.ground().onTrue(getMoveCommand(Position.Ground)); // move to ground
        masher.stow().onTrue(getMoveCommand(Position.Stow)); // move to stow    
        masher.shelf().onTrue(getMoveCommand(Position.Shelf)); // move to shelf

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
        autoChooser.addOption("B-1-1", new AutoRoutines("B-1-1", drive, pivot, telescope, wrist, intake, vision));
        // autoChooser.addOption("R-1-1", new AutoRoutines("R-1-1", drive, pivot, telescope, wrist, intake, vision));
        autoChooser.addOption("B-1-2", new AutoRoutines("B-1-2", drive, pivot, telescope, wrist, intake, vision));
        // autoChooser.addOption("R-1-2", new AutoRoutines("R-1-2", drive, pivot, telescope, wrist, intake, vision));
        autoChooser.addOption("B-2-1", new AutoRoutines("B-2-1", drive, pivot, telescope, wrist, intake, vision));
        // autoChooser.addOption("R-2-1", new AutoRoutines("R-2-1", drive, pivot, telescope, wrist, intake, vision));
        autoChooser.addOption("B-2-2", new AutoRoutines("B-2-2", drive, pivot, telescope, wrist, intake, vision));
        // autoChooser.addOption("R-2-2", new AutoRoutines("R-2-2", drive, pivot, telescope, wrist, intake, vision));
        autoChooser.addOption("B-3-1", new AutoRoutines("B-3-1", drive, pivot, telescope, wrist, intake, vision));
        // autoChooser.addOption("R-3-1", new AutoRoutines("R-3-1", drive, pivot, telescope, wrist, intake, vision));
        autoChooser.addOption("B-3-2", new AutoRoutines("B-3-2", drive, pivot, telescope, wrist, intake, vision));
        // autoChooser.addOption("R-3-2", new AutoRoutines("R-3-2", drive, pivot, telescope, wrist, intake, vision));
        autoChooser.setDefaultOption("B-1-2", new AutoRoutines("B-1-2", drive, pivot, telescope, wrist, intake, vision));
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

    public Command getMoveCommand(Position position) {

        return new InstantCommand(() -> {

            RobotState.getInstance().setStow(position.equals(Position.Stow));
            double[] positions = PositionHelper.getDouble(position, RobotState.getInstance().getMode());

            positions = ArmPositions.intakeCubeExtendedGround;

            new MovePivot(pivot, positions[0]).schedule();
            new MoveTelescope(telescope, pivot, positions[1], positions[0]).schedule();
            new MoveWrist(wrist, pivot, positions[2]).schedule();

            /*
            if (telescope.getPositionM() > 0.2 && position != Position.Stow) {
                new ParallelCommandGroup(
                    new MovePivot(pivot, ArmPositions.stow[0]),
                    new MoveTelescope(telescope, pivot, ArmPositions.stow[1]),
                    new MoveWrist(wrist, pivot, ArmPositions.stow[2])
                ).andThen(
                    new ParallelCommandGroup(
                        new MovePivot(pivot, positions[0]),
                        new MoveTelescope(telescope, pivot, positions[1]),
                        new MoveWrist(wrist, pivot, positions[2])
                    )
                ).schedule();
            }
            else {
                new MovePivot(pivot, positions[0]).schedule();
                new MoveTelescope(telescope, pivot, positions[1]).schedule();
                new MoveWrist(wrist, pivot, positions[2]).schedule();
            }

            */

        });

    }
}