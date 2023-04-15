// <https://www.youtube.com/watch?v=mXlsbBgrKKY>
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ArmPositions;
import frc.robot.Constants.DIOPorts;
import frc.robot.Constants.GamePieceMode;
import frc.robot.Constants.Position;
import frc.robot.commands.CharacterizeMechanism;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.auto.Align;
import frc.robot.commands.auto.AutoRoutines;
import frc.robot.commands.auto.Balance;
import frc.robot.commands.auto.FollowTrajectory;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDManager;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
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

    SendableChooser<String> autoChooser = new SendableChooser<String>();
    private Command activeAutoCommand = null;
    private String activeAutoName = null;

    private DigitalInput brakeSwitch = new DigitalInput(DIOPorts.switch1);
    private DigitalInput ledsSwitch = new DigitalInput(DIOPorts.switch2);

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
        // wrist.setDefaultCommand(new CharacterizeMechanism(wrist, masher.getGamepad(), (v) -> wrist.overrideVolts(v), 0.3));
        // telescope.setDefaultCommand(new CharacterizeMechanism(telescope, masher.getGamepad(), (p) -> telescope.setP(p), 5));
        wrist.setDefaultCommand(new HoldWrist(wrist, pivot));

    }

    private void configureTestBindings() {

        masher.high().onTrue(new MoveWrist(wrist, pivot, 1.56));
        masher.ground().onTrue(new MoveWrist(wrist, pivot, 0));

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

    }   

    private void configureCompBindings() {
        
        // Drive
        new JoystickButton(rightStick, 1)
            .onTrue(new InstantCommand(() -> RobotState.getInstance().invertBack()));

        new JoystickButton(rightStick, 2)
            .onTrue(new InstantCommand(() -> {
                drive.resetHeading(); 
                drive.setFieldToVehicle(new Pose2d(RobotState.getInstance().getFieldToVehicle().getTranslation(), new Rotation2d(0)));
            }));

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

        new JoystickButton(leftStick, 14)
            .onTrue(new InstantCommand(drive::toggleKillFrontRight));

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

        masher.homeWrist()
            .onTrue(new HomeWrist(wrist));
        }

    private void configureAutos() {
        autoChooser.addOption("B-1-1", "B-1-1");
        autoChooser.addOption("B-1-2", "B-1-2");
        autoChooser.addOption("B-1-3", "B-1-3");

        autoChooser.addOption("B-2-1", "B-2-1");

        // autoChooser.addOption("B-3-1", "B-3-1");
        // autoChooser.addOption("B-3-2", "B-3-2");
        // autoChooser.addOption("R-1-1", "R-1-1");
        // autoChooser.addOption("R-1-2", "R-1-2");
        // autoChooser.addOption("R-3-1", "R-3-1");
        // autoChooser.addOption("R-3-2", "R-3-2");
        
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

        if (DriverStation.isTeleop()) {
            new HomeTelescope(telescope).schedule();
            if (!RobotState.getInstance().hasIntaked()) {
                new HomeWrist(wrist).schedule();
            }
        }

        pivot.setDesiredSetpointRad(new State(Math.PI / 2, 0));
        wrist.updateDesiredSetpointRad(new State(Math.PI / 2, 0));
        telescope.setDesiredSetpoint(new State(0.1, 0));
    }

    private Command getMoveArmCommand(Position position) {
        return new InstantCommand(() -> {{
            
            RobotState.getInstance().setStow(position.equals(Position.Stow));
            double[] positions = PositionHelper.getDouble(position, RobotState.getInstance().getMode());
            
            if (position == Position.High || position == Position.Mid) {
                boolean atBack = Math.abs(drive.getRotation().getRadians()) < Math.PI / 2;
                RobotState.getInstance().setBack(atBack);
            }
            else if (position == Position.Shelf) {
                boolean atBack = Math.abs(drive.getRotation().getRadians()) > Math.PI / 2;
                RobotState.getInstance().setBack(atBack);
            }
            
            Timer timer = new Timer();
            timer.reset();
            timer.start();

            if (telescope.getPositionM() > 0.15) {
                // move telescope to 0.05, then move pivot and wrist, then move telescope to goal
                new SequentialCommandGroup(
                    new ParallelRaceGroup(
                        new HoldPivot(pivot, telescope),
                        new MoveTelescope(telescope, pivot, 0.05, true),
                        new HoldWrist(wrist, pivot)
                    ),
                    new ParallelRaceGroup(
                        new MovePivot(pivot, positions[0], true).andThen(new HoldPivot(pivot, telescope)),
                        new HoldTelescope(telescope, pivot),
                        new MoveWrist(wrist, pivot, positions[2], true).andThen(new HoldWrist(wrist, pivot)),
                        new WaitUntilCommand(() -> (pivot.atGoal && wrist.atGoal))
                    ),
                    new ParallelRaceGroup(
                        new MovePivot(pivot, positions[0], false).andThen(new HoldPivot(pivot, telescope)),
                        new MoveTelescope(telescope, pivot, positions[1], false).andThen(new HoldTelescope(telescope, pivot)),
                        new MoveWrist(wrist, pivot, positions[2], false).andThen(new HoldWrist(wrist, pivot)),
                        new WaitUntilCommand(() -> (telescope.atGoal && pivot.atGoal && wrist.atGoal))
                    )
                ).schedule();
            }
            else {
                // move
                new MovePivot(pivot, positions[0]).schedule();
                new MoveTelescope(telescope, pivot, positions[1]).schedule();
                new MoveWrist(wrist, pivot, positions[2]).schedule();
            }

            // SmartDashboard.putNumber("MoveInitTime", timer.get()*1000);
            
        }});
    }

}
// </https://www.youtube.com/watch?v=mXlsbBgrKKY>