// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmPositions;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GamePieceMode;
import frc.robot.Constants.Position;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDManager;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AutoManager;
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
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class RobotContainer {
    
    private final Drive drive = new Drive();
    private final PivotSubsystem pivot = new PivotSubsystem();
    private final TelescopeSubsystem telescope = new TelescopeSubsystem();
    private final WristSubsystem wrist = new WristSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final Vision vision = new Vision();
    private final LEDManager ledManager = new LEDManager();
    private final AutoManager autoManager = new AutoManager(pivot, telescope, wrist, drive, intake, (position) -> getMoveCommand(position));

    private final Joystick leftStick = new Joystick(0);
    private final Joystick rightStick = new Joystick(1);
    private final ButtonMasher masher = new XboxMasher(new XboxController(2));

    private final XboxController gamepad = new XboxController(2);

    private Command[] autoPathCommands = new Command[9];
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

            //place cube
        new JoystickButton(gamepad, Button.kA.value)
            .onTrue(
                autoManager.getEventMap().get("PlaceCube")
            );
        
            //place cone
        new JoystickButton(gamepad, Button.kB.value)
            .onTrue(
                autoManager.getEventMap().get("PlaceCone")
            );

        new JoystickButton(gamepad, Button.kX.value)
            .onTrue(
                autoManager.getEventMap().get("PreparePlaceCone")
            );

            // stow
        new JoystickButton(gamepad, Button.kY.value)
            .onTrue(
                autoManager.getEventMap().get("Stow")
            );

        new JoystickButton(gamepad, Button.kRightBumper.value)
            .onTrue(
                autoManager.getEventMap().get("PickupCone")
            );

        new JoystickButton(gamepad, Button.kStart.value)
            .onTrue(
                autoManager.getEventMap().get("Invert")
            );
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

        new JoystickButton(leftStick, 8) 
            .onTrue(new InstantCommand(() -> gamepad.setRumble(RumbleType.kBothRumble, 0.500)))
            .onFalse(new InstantCommand(() -> gamepad.setRumble(RumbleType.kBothRumble, 0)));

        new JoystickButton(leftStick, 10)
            .onTrue(new HomeWrist(wrist));

             
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
        // Load path groups
        ArrayList<List<PathPlannerTrajectory>> pathGroups = new ArrayList<List<PathPlannerTrajectory>>(9);
        for (int start = 1; start <= 3; start++) {
            for (int path = 1; path <= 3; path++) {
                pathGroups.add(PathPlanner.loadPathGroup(start+"-"+path, new PathConstraints(AutoConstants.kMaxVelocityMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared)));
            }
        }

        Supplier<Pose2d> poseSupplier = () -> RobotState.getInstance().getFieldToVehicle();
		Consumer<Pose2d> poseConsumer = fieldToVehicle -> drive.setFieldToVehicle(fieldToVehicle);
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
			poseSupplier, // Pose2d supplier
			poseConsumer, // Pose2d consumer, used to reset odometry at the beginning of auto
			new PIDConstants(AutoConstants.autoTranslationKp, AutoConstants.autoTranslationKi, AutoConstants.autoTranslationKd), // PID constants to correct for translation error (used to create the X and Y PID controllers)
			new PIDConstants(AutoConstants.autoRotationKp, AutoConstants.autoRotationKi, AutoConstants.autoRotationKd), // PID constants to correct for rotation error (used to create the rotation controller)
			//moduleStates -> drive.setGoalModuleStates(moduleStates), // Module states consumer used to output to the drive subsystem
            drive::setGoalChassisSpeeds,
			autoManager.getEventMap(), // The events mapped to commands
            true,
			drive // The drive subsystem. Used to properly set the requirements of path following commands
		);

        for (int i = 0; i < 9; i++){
            autoPathCommands[i] = autoBuilder.fullAuto(pathGroups.get(i)).andThen(() -> drive.setGoalChassisSpeeds(new ChassisSpeeds(0, 0, 0)));
        }

        // Select path
        int i = 0;
        for (int start = 1; start <= 3; start++) {
            for (int path = 1; path <= 3; path++) {
                autoChooser.addOption(start+"-"+path, autoPathCommands[i++]);
            }
        }
        autoChooser.setDefaultOption("1-1", autoPathCommands[0]);
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    public Command getAutonomousCommand() {
        SmartDashboard.putNumber("JAKLJDFKLAD", System.currentTimeMillis());
        return autoChooser.getSelected();//new WaitCommand(3).andThen(autoManager.getEventMap().get("PlaceCube"));//
    }

    public void enabledInit() {
        if (!telescope.homed && DriverStation.isTeleop()) {
            new HomeTelescope(telescope).schedule();
        }

        if (!wrist.homed && DriverStation.isTeleop()) {
            new HomeWrist(wrist).schedule();
        }
        else if (!wrist.homed) {
            wrist.zeroOffset();
            wrist.homed = true;
        }

        // pivot.setDesiredSetpointRad(new TrapezoidProfile.State(pivot.getPositionRad(), 0));
    }

    public Command getMoveCommand(Position position) {

        return new InstantCommand(() -> {
            RobotState.getInstance().setStow(position.equals(Position.Stow));
            double[] positions = PositionHelper.getDouble(position, RobotState.getInstance().getMode());
            new MovePivot(pivot,telescope,() -> positions[0]).schedule();
            new MoveTelescope(telescope, pivot, () -> positions[1],() -> positions[0]).schedule();
            new MoveWrist(wrist, pivot, () -> positions[2]).schedule();
        });

    }
}
