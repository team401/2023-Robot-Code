// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmPositions;
import frc.robot.Constants.DIOPorts;
import frc.robot.Constants.Position;
// import frc.robot.commands.pivot.HoldPivot;
// import frc.robot.commands.pivot.MovePivot;
// import frc.robot.commands.telescope.HoldTelescope;
// import frc.robot.commands.telescope.HomeTelescope;
// import frc.robot.commands.telescope.MoveTelescope;
// import frc.robot.commands.wrist.HoldWrist;
// import frc.robot.commands.wrist.HomeWrist;
// import frc.robot.commands.wrist.MoveWrist;
import frc.robot.generated.TunerConstants;
// import frc.robot.subsystems.IntakeSubsystem;
// import frc.robot.subsystems.PivotSubsystem;
// import frc.robot.subsystems.TelescopeSubsystem;
// import frc.robot.subsystems.WristSubsystem;
// import frc.robot.util.PositionHelper;

public class RobotContainer {
    final double MaxSpeed = 6; // 6 meters per second desired top speed
    final double MaxAngularRate = Math.PI * 1.5; // Half a rotation per second max angular velocity
    final double deadbandPercent = 0.5;

    /* Setting up bindings for necessary control of the swerve drive platform */
    CommandJoystick leftJoystick = new CommandJoystick(0); // Left joystick
    CommandJoystick rightJoystick = new CommandJoystick(1); // Right joystick
    CommandXboxController controller = new CommandXboxController(2); // Button Masher
    CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
    SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withIsOpenLoop(true).withDeadband(deadbandPercent); // I want field-centric
                                                                                              // driving in open loop
    SwerveRequest.RobotCentric driveRobot = new SwerveRequest.RobotCentric().withIsOpenLoop(true).withDeadband(deadbandPercent); // I want robot
                                                                                                   // centric
                                                                                                   // driving in open
                                                                                                   // loop
    SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric();
    SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    // private final PivotSubsystem pivot = new PivotSubsystem();
    // private final TelescopeSubsystem telescope = new TelescopeSubsystem();
    // private final WristSubsystem wrist = new WristSubsystem();
    // private final IntakeSubsystem intake = new IntakeSubsystem();

    Telemetry logger = new Telemetry(MaxSpeed);

    private DigitalInput brakeSwitch = new DigitalInput(DIOPorts.switch1);
    private DigitalInput ledsSwitch = new DigitalInput(DIOPorts.switch2);

    private void configureBindings() {
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> drive.withVelocityX(-leftJoystick.getY() * MaxSpeed)
                        .withVelocityY(-leftJoystick.getX() * MaxSpeed)
                        .withRotationalRate(-rightJoystick.getX() * MaxAngularRate)));

        rightJoystick.trigger()
                .whileTrue(drivetrain.applyRequest(() -> driveRobot.withVelocityX(-leftJoystick.getY() * MaxSpeed)
                        .withVelocityY(-leftJoystick.getX() * MaxSpeed)
                        .withRotationalRate(-rightJoystick.getX() * MaxAngularRate)));

        leftJoystick.trigger()
                .whileTrue(drivetrain.applyRequest(() -> drive.withVelocityX(-leftJoystick.getY() * MaxSpeed * 0.5)
                        .withVelocityY(-leftJoystick.getX() * MaxSpeed * 0.5)
                        .withRotationalRate(-rightJoystick.getX() * MaxAngularRate * 0.5)));

        rightJoystick.button(2)
                .whileTrue(new InstantCommand(() -> drivetrain.seedFieldRelative()));

        controller.a().whileTrue(drivetrain.applyRequest(() -> brake));
        controller.b().whileTrue(drivetrain
                .applyRequest(
                        () -> point
                                .withModuleDirection(new Rotation2d(-controller.getLeftY(), -controller.getLeftX()))));

        controller.x().whileTrue(drivetrain.applyRequest(() -> drive.withIsOpenLoop(false)));

        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public void configureSubsystems() {
        // pivot.setDefaultCommand(new HoldPivot(pivot, telescope));
        // telescope.setDefaultCommand(new HoldTelescope(telescope, pivot));
        // wrist.setDefaultCommand(new HoldWrist(wrist, pivot));
    }

    public RobotContainer() {
        configureBindings();
        configureSubsystems();
    }

    public void disabledPeriodic() {
        // pivot.setBrakeMode(!brakeSwitch.get());
        // telescope.setBrakeMode(!brakeSwitch.get());
        // wrist.setBrakeMode(!brakeSwitch.get());

    }

    public void enabledInit() {
        // pivot.setBrakeMode(true);
        // telescope.setBrakeMode(true);
        // wrist.setBrakeMode(true);

        if (DriverStation.isTeleop()) {
            // new HomeTelescope(telescope).schedule();
            if (!RobotState.getInstance().hasIntaked()) {
                // new HomeWrist(wrist).schedule();
            }
            // pivot.normalConstrain();
        }

        // pivot.setDesiredSetpointRad(new State(ArmPositions.stow[0], 0));
        // telescope.setDesiredSetpoint(new State(ArmPositions.stow[1], 0));
        // wrist.updateDesiredSetpointRad(new State(ArmPositions.stow[2], 0));
    }

    // private Command getMoveArmCommand(Position position) {
    // return new InstantCommand(() -> {

    // RobotState.getInstance().setStow(position == Position.Stow);
    // double[] positions = PositionHelper.getDouble(position,
    // RobotState.getInstance().getMode());

    // if (position == Position.High || position == Position.Mid) {
    // boolean atBack = Math.abs(logger.getRotationRadians()) < Math.PI / 2;
    // RobotState.getInstance().setBack(atBack);
    // } else if (position == Position.Shelf) {
    // boolean atBack = Math.abs(logger.getRotationRadians()) > Math.PI / 2;
    // RobotState.getInstance().setBack(atBack);
    // }

    // Timer timer = new Timer();
    // timer.reset();
    // timer.start();

    // if (telescope.getPositionM() > 0.15 || positions[1] > 0.15) {
    // // move telescope to 0.05, then move pivot and wrist, then move telescope to
    // // goal
    // new SequentialCommandGroup(
    // new ParallelRaceGroup(
    // new HoldPivot(pivot, telescope),
    // new MoveTelescope(telescope, pivot, 0.05, true),
    // new HoldWrist(wrist, pivot)),
    // new ParallelRaceGroup(
    // new MovePivot(pivot, positions[0], true).andThen(new HoldPivot(pivot,
    // telescope)),
    // new HoldTelescope(telescope, pivot),
    // new MoveWrist(wrist, pivot, positions[2], true).andThen(new HoldWrist(wrist,
    // pivot)),
    // new WaitUntilCommand(() -> (pivot.atGoal && wrist.atGoal))),
    // new ParallelRaceGroup(
    // new MovePivot(pivot, positions[0], false).andThen(new HoldPivot(pivot,
    // telescope)),
    // new MoveTelescope(telescope, pivot, positions[1], false)
    // .andThen(new HoldTelescope(telescope, pivot)),
    // new MoveWrist(wrist, pivot, positions[2], false).andThen(new HoldWrist(wrist,
    // pivot)),
    // new WaitUntilCommand(() -> (telescope.atGoal && pivot.atGoal &&
    // wrist.atGoal))))
    // .schedule();
    // } else {
    // // move
    // new MovePivot(pivot, positions[0]).schedule();
    // new MoveTelescope(telescope, pivot, positions[1]).schedule();
    // new MoveWrist(wrist, pivot, positions[2]).schedule();
    // }
    // });
    // }
}
