// <https://www.youtube.com/watch?v=mXlsbBgrKKY>
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ArmPositions;
import frc.robot.Constants.DIOPorts;
import frc.robot.Constants.DriveModulePosition;
import frc.robot.Constants.GamePieceMode;
import frc.robot.Constants.Position;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.auto.Align;
import frc.robot.commands.auto.AutoRoutines;
import frc.robot.subsystems.LEDManager;
import frc.robot.subsystems.drive.AngleIO;
import frc.robot.subsystems.drive.AngleIOPidgeon2;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOFalcon500;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.subsystems.pivot.PivotIOFalcon;
import frc.robot.subsystems.pivot.PivotIOSim;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.telescope.TelescopeIO;
import frc.robot.subsystems.telescope.TelescopeIOFalcon;
import frc.robot.subsystems.telescope.TelescopeIOSim;
import frc.robot.subsystems.telescope.TelescopeSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.wrist.WristIO;
import frc.robot.subsystems.wrist.WristIOSim;
import frc.robot.subsystems.wrist.WristIOFalcon;
import frc.robot.subsystems.wrist.WristSubsystem;
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

public class RobotContainer {
    
    private final Drive drive;
    private final PivotSubsystem pivot;
    private final TelescopeSubsystem telescope;
    private final WristSubsystem wrist;
    private final IntakeSubsystem intake;
    @SuppressWarnings("unused")
    private final Vision vision;
    private final LEDManager ledManager = new LEDManager();

    private final Joystick leftStick = new Joystick(0);
    private final Joystick rightStick = new Joystick(1);
    private final ButtonMasher masher = new ButtonMasher(2);

    private final PS4Controller controller = new PS4Controller(3);

    SendableChooser<String> autoChooser = new SendableChooser<String>();
    private Command activeAutoCommand = null;
    private String activeAutoName = null;

    private DigitalInput brakeSwitch = new DigitalInput(DIOPorts.switch1);
    private DigitalInput ledsSwitch = new DigitalInput(DIOPorts.switch2);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        switch(Constants.mode) {
            case REAL:
                drive = new Drive(
                    new AngleIOPidgeon2(),
                    new ModuleIOFalcon500(DriveModulePosition.FRONT_LEFT),
                    new ModuleIOFalcon500(DriveModulePosition.FRONT_RIGHT),
                    new ModuleIOFalcon500(DriveModulePosition.BACK_LEFT),
                    new ModuleIOFalcon500(DriveModulePosition.BACK_RIGHT));
                pivot = new PivotSubsystem(new PivotIOFalcon());
                telescope = new TelescopeSubsystem(new TelescopeIOFalcon());
                wrist = new WristSubsystem(new WristIOFalcon());
                intake = new IntakeSubsystem(new IntakeIOSparkMax());
                break;
            case SIM:
                drive = new Drive(
                    new AngleIO() {},
                    new ModuleIOSim(),
                    new ModuleIOSim(),
                    new ModuleIOSim(),
                    new ModuleIOSim());
                pivot = new PivotSubsystem(new PivotIOSim());
                telescope = new TelescopeSubsystem(new TelescopeIOSim());
                wrist = new WristSubsystem(new WristIOSim());
                intake = new IntakeSubsystem(new IntakeIOSim());
                break;
            default:
                drive = new Drive(
                    new AngleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {});
                pivot = new PivotSubsystem(new PivotIO() {});
                telescope = new TelescopeSubsystem(new TelescopeIO() {});
                wrist = new WristSubsystem(new WristIO() {});
                intake = new IntakeSubsystem(new IntakeIO() {});
                break;
        }

        vision = new Vision(drive::recordVisionObservations);


        configureSubsystems();
        // configureCompBindings();
        configureTestBindings();
        configureAutos();
    }

    private void configureSubsystems() {

        // drive.setDefaultCommand(new DriveWithJoysticks(
        //     drive,
        //     () -> -leftStick.getRawAxis(1),
        //     () -> -leftStick.getRawAxis(0),
        //     () -> -rightStick.getRawAxis(0),
        //     true
        // ));

        drive.setDefaultCommand(new DriveWithJoysticks(
            drive,
            () -> controller.getLeftX(),
            () -> controller.getLeftY(),
            () -> controller.getRawAxis(3),
            false));

        pivot.setDefaultCommand(new HoldPivot(pivot, telescope));
        telescope.setDefaultCommand(new HoldTelescope(telescope, pivot));
        wrist.setDefaultCommand(new HoldWrist(wrist, pivot));

    }

    @SuppressWarnings("unused")
    private void configureTestBindings() {
        // masher.a().onTrue(new InstantCommand(() -> wrist.setVolts(2)));

        // masher.y().onTrue(new InstantCommand(() -> wrist.setVolts(0)));

        masher.b().onTrue(new InstantCommand(() -> wrist.jogSetpointForward(), wrist));

        masher.x().onTrue(new InstantCommand(() -> wrist.jogSetpointBack(), wrist));
    }   

    @SuppressWarnings("unused")
    private void configureCompBindings() {
        
        // Drive
        new JoystickButton(rightStick, 1)
            .onTrue(new InstantCommand(() -> RobotState.getInstance().invertBack()));

        new JoystickButton(rightStick, 2)
            .onTrue(new InstantCommand(() -> drive.resetHeading()));

        new JoystickButton(leftStick, 1)
            .onTrue(new InstantCommand(() -> drive.setBabyMode(true)))
            .onFalse(new InstantCommand(() -> drive.setBabyMode(false)));

        new JoystickButton(leftStick, 3)
            .whileTrue(new Align(drive, true));
        new JoystickButton(leftStick, 4)
            .whileTrue(new Align(drive, false));

        // Overrides
        new JoystickButton(rightStick, 12)
            .onTrue(new HomeWrist(wrist));
        
        new JoystickButton(rightStick, 13)
            .onTrue(new HomeTelescope(telescope));
        
        new JoystickButton(rightStick, 15)
            .whileTrue(new RunCommand(() -> pivot.overrideVolts(12), pivot));

        new JoystickButton(rightStick, 14)
            .whileTrue(new RunCommand(() -> pivot.overrideVolts(-12), pivot));

        new JoystickButton(rightStick, 11)
            .whileTrue(new RunCommand(() -> wrist.overrideVolts(5), wrist));

        new JoystickButton(rightStick, 16)
            .whileTrue(new RunCommand(() -> wrist.overrideVolts(-5), wrist));

        new JoystickButton(leftStick, 7)
            .onTrue(new InstantCommand(pivot::toggleKill, pivot));

        new JoystickButton(leftStick, 6)
            .onTrue(new InstantCommand(telescope::toggleKill, telescope));

        new JoystickButton(leftStick, 5)
            .onTrue(new InstantCommand(wrist::toggleKill, wrist));

        new JoystickButton(leftStick, 8)
            .onTrue(new InstantCommand(pivot::toggleKill))
            .onTrue(new InstantCommand(telescope::toggleKill))
            .onTrue(new InstantCommand(wrist::toggleKill));

        new JoystickButton(leftStick,12)
            .onTrue(new InstantCommand(() -> drive.toggleKill(0)));
        new JoystickButton(leftStick,13)
            .onTrue(new InstantCommand(() -> drive.toggleKill(1)));
        new JoystickButton(leftStick,15)
            .onTrue(new InstantCommand(() -> drive.toggleKill(2)));
        new JoystickButton(leftStick,14)
            .onTrue(new InstantCommand(() -> drive.toggleKill(3)));

        // Set game piece mode
        masher.leftBumper().onTrue(new InstantCommand(() ->
            RobotState.getInstance().setMode(GamePieceMode.Cube)));           
        masher.rightBumper().onTrue(new InstantCommand(() ->
            RobotState.getInstance().setMode(GamePieceMode.ConeDown)));
        masher.y().onTrue(new InstantCommand(() ->
            RobotState.getInstance().setMode(GamePieceMode.ConeUp)));
        
        // Move arm
        masher.pov(0).onTrue(getMoveArmCommand(Position.High)); // move to high
        masher.pov(90).onTrue(getMoveArmCommand(Position.Mid)); // move to mid
        masher.pov(180).onTrue(getMoveArmCommand(Position.Ground)); // move to ground
        masher.pov(270).onTrue(getMoveArmCommand(Position.Stow)); // move to stow    
        masher.x().onTrue(getMoveArmCommand(Position.Shelf)); // move to shelf

        masher.a().onTrue(new MoveWrist(wrist, pivot, ArmPositions.wristConePlace)); // Dunk cone
        masher.b().onTrue(new MoveWrist(wrist, pivot, ArmPositions.placeConeDownHigh[2])); // Un-dunk cone
        
        // TODO: set button
        // masher.??????().onTrue(
        //     new InstantCommand(() -> RobotState.getInstance().invertBack())); // flip side

        // Manually jog arm
        masher.leftStickLeft()
            .onTrue(new InstantCommand(pivot::jogSetpointForward, pivot)); // jog pivot right
        masher.leftStickRight()
            .onTrue(new InstantCommand(pivot::jogSetpointBack, pivot)); // jog pivot left

        masher.rightStickRight()
            .onTrue(new InstantCommand(telescope::jogSetpointForward, telescope)); // jog telescope up
        masher.rightStickLeft()
            .onTrue(new InstantCommand(telescope::jogSetpointBackward, telescope)); // jog telescope down

        masher.rightStickUp()
            .onTrue(new InstantCommand(wrist::jogSetpointForward, wrist)); // jog wrist right
        masher.rightStickDown()
            .onTrue(new InstantCommand(wrist::jogSetpointBack, wrist)); // jog wrist left

        // Intake
        masher.leftTrigger()
            .onTrue(new InstantCommand(intake::toggleIntake)); // toggle intake

        masher.rightTrigger()
            .onTrue(new InstantCommand(intake::place)) // start place
            .onFalse(new InstantCommand(intake::stop)); // stop place

        masher.start()
            .onTrue(new HomeWrist(wrist));
        }

    private void configureAutos() {
        autoChooser.addOption("B-1-1", "B-1-1");
        autoChooser.addOption("B-1-2", "B-1-2");
        autoChooser.addOption("B-1-3", "B-1-3");

        autoChooser.addOption("B-2-1", "B-2-1");

        autoChooser.addOption("B-3-1", "B-3-1");
        autoChooser.addOption("B-3-2", "B-3-2");

        autoChooser.addOption("R-1-1", "R-1-1");
        autoChooser.addOption("R-1-2", "R-1-2");
        autoChooser.addOption("R-1-3", "R-1-3");

        autoChooser.addOption("R-2-1", "R-2-1");

        autoChooser.addOption("R-3-1", "R-3-1");
        autoChooser.addOption("R-3-2", "R-3-2");
        
        autoChooser.setDefaultOption("default", "B-1-1");
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    public Command getAutonomousCommand() {
        return activeAutoCommand;
    }

    public void disabledPeriodic() {
        if (activeAutoCommand == null || !activeAutoName.equals(autoChooser.getSelected())) {
            activeAutoCommand = new AutoRoutines(autoChooser.getSelected(), drive, pivot, telescope, wrist, intake);
            activeAutoName = autoChooser.getSelected();
        }

        drive.setBrakeMode(!brakeSwitch.get());
        pivot.setBrakeMode(!brakeSwitch.get());
        telescope.setBrakeMode(!brakeSwitch.get());
        wrist.setBrakeMode(!brakeSwitch.get());

        ledManager.setOff(ledsSwitch.get());

    }

    public void enabledInit() {

        drive.setBrakeMode(true);
        pivot.setBrakeMode(true);
        telescope.setBrakeMode(true);
        wrist.setBrakeMode(true);

        ledManager.setOff(false);

        if(Constants.mode == Constants.Mode.REAL) {
            if (DriverStation.isTeleop()) {
                new HomeTelescope(telescope).schedule();
                if (!RobotState.getInstance().hasIntaked()) {
                    new HomeWrist(wrist).schedule();
                }
                pivot.normalConstrain();
            }
        }

        pivot.setDesiredSetpointRad(new State(ArmPositions.stow[0], 0));
        telescope.setDesiredSetpoint(new State(ArmPositions.stow[1], 0));
        wrist.updateDesiredSetpointRad(new State(ArmPositions.stow[2], 0));
    }

    private Command getMoveArmCommand(Position position) {
        //TODO: agressively refactor arm state machine

        RobotState.getInstance().setStow(position == Position.Stow);
        double[] positions = PositionHelper.getDouble(position, RobotState.getInstance().getMode());
        
        // Autoset side
        if (position == Position.High || position == Position.Mid) {
            boolean atBack = Math.abs(drive.getRotation().getRadians()) < Math.PI / 2;
            RobotState.getInstance().setBack(atBack);
        } else if (position == Position.Shelf) {
            boolean atBack = Math.abs(drive.getRotation().getRadians()) > Math.PI / 2;
            RobotState.getInstance().setBack(atBack);
        }
        
        Timer performanceTimer = new Timer();
        performanceTimer.reset();
        performanceTimer.start();

        if (telescope.getPositionM() > 0.15 || positions[1] > 0.15) {
            // move telescope to 0.05, then move pivot and wrist, then move telescope to goal
            return Commands.sequence(
                Commands.race(
                    new HoldPivot(pivot, telescope),
                    new MoveTelescope(telescope, pivot, 0.05, true), // wait for telescope to finish
                    new HoldWrist(wrist, pivot)
                ),
                // this looks weird, I'll test a simplification in shop
                Commands.race(
                    new MovePivot(pivot, positions[0], true).andThen(new HoldPivot(pivot, telescope)),
                    new HoldTelescope(telescope, pivot),
                    new MoveWrist(wrist, pivot, positions[2], true).andThen(new HoldWrist(wrist, pivot)),
                    new WaitUntilCommand(() -> (pivot.atGoal && wrist.atGoal)) // wait for pivot and wrist to finish
                ),
                // I have no idea why this is here
                Commands.race(
                    new MovePivot(pivot, positions[0], false).andThen(new HoldPivot(pivot, telescope)),
                    new MoveTelescope(telescope, pivot, positions[1], false).andThen(new HoldTelescope(telescope, pivot)),
                    new MoveWrist(wrist, pivot, positions[2], false).andThen(new HoldWrist(wrist, pivot)),
                    new WaitUntilCommand(() -> (telescope.atGoal && pivot.atGoal && wrist.atGoal))
                )
            );
        }
        else {
            // move
            return Commands.parallel(
                new MovePivot(pivot, positions[0]),
                new MoveTelescope(telescope, pivot, positions[1]),
                new MoveWrist(wrist, pivot, positions[2]));
        }

        // SmartDashboard.putNumber("MoveInitTime", timer.get()*1000);        
    }

}
// </https://www.youtube.com/watch?v=mXlsbBgrKKY>