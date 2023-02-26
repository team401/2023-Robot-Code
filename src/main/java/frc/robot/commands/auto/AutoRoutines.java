package frc.robot.commands.auto;

import java.util.HashMap;
import java.util.List;
import java.util.function.Function;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotState;
import frc.robot.Constants.ArmPositions;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GamePieceMode;
import frc.robot.Constants.Position;
import frc.robot.commands.pivot.HoldPivot;
import frc.robot.commands.pivot.MovePivot;
import frc.robot.commands.telescope.HoldTelescope;
import frc.robot.commands.telescope.HomeTelescope;
import frc.robot.commands.telescope.MoveTelescope;
import frc.robot.commands.wrist.HoldWrist;
import frc.robot.commands.wrist.HomeWrist;
import frc.robot.commands.wrist.MoveWrist;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.drive.Drive;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

public class AutoRoutines extends SequentialCommandGroup {

    private final Drive drive;
    private final PivotSubsystem pivot;
    private final TelescopeSubsystem telescope;
    private final WristSubsystem wrist;
    private final IntakeSubsystem intake;

    public AutoRoutines(String pathName, Drive driveSubsystem, PivotSubsystem pivotSubsystem, TelescopeSubsystem telescopeSubsystem, WristSubsystem wristSubsystem, IntakeSubsystem intakeSubsystem) {

        drive = driveSubsystem;
        pivot = pivotSubsystem;
        telescope = telescopeSubsystem;
        wrist = wristSubsystem;
        intake = intakeSubsystem;

        // Load path group
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(pathName, new PathConstraints(AutoConstants.kMaxVelocityMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared));
        if (pathGroup.size() == 0) {
            SmartDashboard.putBoolean("FAILED TO LOAD PATH " + pathName, true);
            System.out.println("FAILED TO LOAD PATH " + pathName);
            return;
        }

        if (pathName.equals("1-1") || pathName.equals("3-1")) {
            addCommands(
                new InstantCommand(() -> drive.setFieldToVehicle(new Pose2d(pathGroup.get(0).getInitialPose().getTranslation(), new Rotation2d(0)))),
                home(),
                invert(),
                placeCube(),
                new ParallelRaceGroup(
                    drive( pathGroup.get(0)),
                    ((stow().andThen(invert()).andThen(hold())).raceWith(new WaitCommand(pathGroup.get(0).getTotalTimeSeconds()/3)))
                        .andThen(pickupCone()).andThen(hold())
                ),
                new ParallelRaceGroup(
                    drive(pathGroup.get(1)),
                    ((stow().andThen(hold())).raceWith(new WaitCommand(pathGroup.get(1).getTotalTimeSeconds()*0.7)))
                        .andThen(invert()).andThen(preparePlaceCone()).andThen(hold())
                ),
                placeCone(),
                new ParallelCommandGroup(
                    drive(pathGroup.get(2)).andThen(balance()),
                    stow().andThen(hold())
                )
            );
            
        }
        else if (pathName.equals("1-2") || pathName.equals("3-2")) {
            addCommands(
                new InstantCommand(() -> drive.setFieldToVehicle(new Pose2d(pathGroup.get(0).getInitialPose().getTranslation(), new Rotation2d(Math.PI)))),
                home(),
                placeCube(),
                new ParallelRaceGroup(
                    drive(pathGroup.get(0)),
                    ((stow().andThen(invert()).andThen(hold())).raceWith(new WaitCommand(pathGroup.get(0).getTotalTimeSeconds()/3)))
                        .andThen(pickupCone()).andThen(hold())
                ),
                new ParallelRaceGroup(
                    drive(pathGroup.get(1)),
                    ((stow().andThen(hold())).raceWith(new WaitCommand(pathGroup.get(1).getTotalTimeSeconds()/3)))
                        .andThen(preparePlaceCone()).andThen(hold())
                ),
                placeCone(),
                new ParallelRaceGroup(
                    drive(pathGroup.get(2)),
                    ((stow().andThen(invert()).andThen(hold())).raceWith(new WaitCommand(pathGroup.get(2).getTotalTimeSeconds()/3)))
                        .andThen(pickupCone()).andThen(hold())
                ),
                new ParallelRaceGroup(
                    drive(pathGroup.get(3)),
                    ((stow().andThen(hold())).raceWith(new WaitCommand(pathGroup.get(3).getTotalTimeSeconds()/3)))
                        .andThen(preparePlaceCone()).andThen(hold())
                ),
                placeCone(),
                stow()
            );
        }
        else if (pathName.equals("1-3") || pathName.equals("3-3")) {
            addCommands(
                new InstantCommand(() -> drive.setFieldToVehicle(new Pose2d(pathGroup.get(0).getInitialPose().getTranslation(), new Rotation2d(Math.PI)))),
                home(),
                placeCube(),
                new ParallelRaceGroup(
                    drive(pathGroup.get(0)),
                    ((stow().andThen(invert()).andThen(hold())).raceWith(new WaitCommand(pathGroup.get(0).getTotalTimeSeconds()/3)))
                        .andThen(pickupCone()).andThen(hold())
                ),
                new ParallelRaceGroup(
                    drive(pathGroup.get(1)),
                    ((stow().andThen(hold())).raceWith(new WaitCommand(pathGroup.get(1).getTotalTimeSeconds()/3)))
                        .andThen(preparePlaceCone()).andThen(hold())
                ),
                placeCone(),
                new ParallelRaceGroup(
                    drive(pathGroup.get(2)),
                    ((stow().andThen(invert()).andThen(hold())).raceWith(new WaitCommand(pathGroup.get(2).getTotalTimeSeconds()/3)))
                        .andThen(pickupCone()).andThen(hold())
                ),
                new ParallelRaceGroup(
                    drive(pathGroup.get(3)),
                    ((stow().andThen(hold())).raceWith(new WaitCommand(pathGroup.get(3).getTotalTimeSeconds()/3)))
                        .andThen(preparePlaceCone()).andThen(hold())
                ),
                placeCone(),
                new ParallelCommandGroup(
                    drive(pathGroup.get(4)).andThen(balance()),
                    stow().andThen(hold())
                )
            );
        }

    }

    private Command drive(PathPlannerTrajectory trajectory) {
        return new SequentialCommandGroup(
            followPath(drive, trajectory),
            new InstantCommand(() -> drive.setGoalChassisSpeeds(new ChassisSpeeds(0, 0, 0)))
        );
    }


    private Command placeCube() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> RobotState.getInstance().setMode(GamePieceMode.Cube)),
            new MovePivot(pivot, telescope, () -> ArmPositions.placeCubeHigh[0])
                .alongWith(new MoveTelescope(telescope, pivot, () -> ArmPositions.placeCubeHigh[1],() -> ArmPositions.placeCubeHigh[0]))
                .alongWith(new MoveWrist(wrist, pivot, () -> ArmPositions.placeCubeHigh[2])),
            stow().raceWith(new SequentialCommandGroup(
                new InstantCommand(intake::place),
                new WaitCommand(0.25),
                new InstantCommand(intake::stopMotor)
            ))
        );
    }

    public Command placeCone() {
        return new SequentialCommandGroup(
            new MoveWrist(wrist, pivot, () -> ArmPositions.wristConePlace),
            new InstantCommand(intake::stopMotor)
        );
    }

    public Command preparePlaceCone() {
        return new ParallelCommandGroup(
            new MovePivot(pivot, telescope, () -> ArmPositions.placeConeBackHigh[0]),
            new MoveTelescope(telescope, pivot, () -> ArmPositions.placeConeBackHigh[1], () -> ArmPositions.placeConeBackHigh[0]),
            new MoveWrist(wrist, pivot, () -> ArmPositions.placeConeBackHigh[2])
        );
    }

    public Command pickupCone() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> RobotState.getInstance().setMode(GamePieceMode.ConeBack)),
            new ParallelCommandGroup(
                new MovePivot(pivot, telescope, () -> ArmPositions.intakeConeBackGround[0]),
                new MoveTelescope(telescope, pivot, () -> ArmPositions.intakeConeBackGround[1], () -> ArmPositions.intakeConeBackGround[0]),
                new MoveWrist(wrist, pivot, () -> ArmPositions.intakeConeBackGround[2])
            ),
            new InstantCommand(intake::intake)
        );
    }

    public Command hold() {
        return new ParallelCommandGroup(
            new HoldPivot(pivot, telescope),
            new HoldTelescope(telescope, pivot),
            new HoldWrist(wrist, pivot)
        );
    }

    public Command invert() {
        return new InstantCommand(() -> RobotState.getInstance().invertBack());
    }

    public Command balance() {
        return new InstantCommand();
    }

    public Command stow() {
        return new ParallelCommandGroup(
            new MovePivot(pivot, telescope, () -> ArmPositions.stow[0]),
            new MoveTelescope(telescope, pivot, () -> ArmPositions.stow[1], () -> ArmPositions.stow[0]),
            new MoveWrist(wrist, pivot, () -> ArmPositions.stow[2])
        );
    }

    public Command home() {
        return new ParallelCommandGroup(
            new HomeTelescope(telescope),
            new InstantCommand(() -> { wrist.zeroOffset(); wrist.homed = true; })
        );
    }

    private Command followPath(Drive drive, PathPlannerTrajectory trajectory) {

        return new PPSwerveControllerCommand(
            trajectory, 
            () -> RobotState.getInstance().getFieldToVehicle(), // Pose supplier
            new PIDController(AutoConstants.autoTranslationKp, AutoConstants.autoTranslationKi, AutoConstants.autoTranslationKd), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            new PIDController(AutoConstants.autoTranslationKp, AutoConstants.autoTranslationKi, AutoConstants.autoTranslationKd), // Y controller (usually the same values as X controller)
            new PIDController(AutoConstants.autoRotationKp, AutoConstants.autoRotationKi, AutoConstants.autoRotationKd), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            drive::setGoalChassisSpeeds, // Module states consumer
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            drive // Requires this drive subsystem
        );
    }

}

/*private PivotSubsystem pivot; 
    private TelescopeSubsystem telescope; 
    private WristSubsystem wrist; 
    private Drive drive; 
    private IntakeSubsystem intake;
    private HashMap<String, Command> eventMap;
    private Function<Position, Command> getMoveCommand;

    public AutoManager(PivotSubsystem pivot, TelescopeSubsystem telescope, WristSubsystem wrist, Drive drive, IntakeSubsystem intake, Function<Position, Command> function) {
        this.pivot = pivot; this.telescope = telescope; this.wrist = wrist; this.drive = drive; this.intake = intake; getMoveCommand = function;

        eventMap = new HashMap<String, Command>() {{
            put("Home", new HomeTelescope(telescope));
            put("PlaceCube", new SequentialCommandGroup(
                new InstantCommand(() -> SmartDashboard.putNumber("LMAOXD", 3)),
                new InstantCommand(() -> RobotState.getInstance().setMode(GamePieceMode.Cube)),
                new MovePivot(pivot, telescope, () -> ArmPositions.placeCubeHigh[0])
                    .alongWith(new MoveTelescope(telescope, pivot, () -> ArmPositions.placeCubeHigh[1],() -> ArmPositions.placeCubeHigh[0]))
                    .alongWith(new MoveWrist(wrist, pivot, () -> ArmPositions.placeCubeHigh[2])),
                new InstantCommand(() -> SmartDashboard.putNumber("LMAOXD", 2)),
                new InstantCommand(intake::place),
                new WaitCommand(0.25),
                new InstantCommand(intake::stopMotor),
                new InstantCommand(() -> SmartDashboard.putNumber("EndPlaceCubeTime", System.currentTimeMillis()))
            ));
            put("PlaceCone", new SequentialCommandGroup(
                new MoveWrist(wrist, pivot, () -> ArmPositions.wristConePlace),
                new InstantCommand(intake::stopMotor)
            ));
            put("PreparePlaceCone", new InstantCommand(
                () -> getMoveCommand.apply(Position.High).schedule()
            ));
            put("PickupCone", new SequentialCommandGroup(
                new InstantCommand(() -> RobotState.getInstance().setMode(GamePieceMode.ConeBack)),
                new InstantCommand(() -> getMoveCommand.apply(Position.Ground).schedule()),
                new InstantCommand(intake::intake)
            ));
            put("Stow", new InstantCommand( () -> {
                    new MovePivot(pivot, telescope, () -> ArmPositions.stow[0]).schedule();
                    new MoveTelescope(telescope, pivot, () -> ArmPositions.stow[1],() -> ArmPositions.stow[0]).schedule();
                    new MoveWrist(wrist, pivot, () -> ArmPositions.stow[2]).schedule();
                }));//new ParallelCommandGroup(
            //     new MovePivot(pivot, telescope, () -> ArmPositions.stow[0]),
            //     new MoveTelescope(telescope, pivot, () -> ArmPositions.stow[1],() -> ArmPositions.stow[0]),
            //     new MoveWrist(wrist, pivot, () -> ArmPositions.stow[2])
            // ));
            // new InstantCommand( () -> {
            //     // new MovePivot(pivot, telescope, () -> ArmPositions.stow[0]).schedule();
            //     // new MoveTelescope(telescope, pivot, () -> ArmPositions.stow[1],() -> ArmPositions.stow[0]).schedule();
            //     // new MoveWrist(wrist, pivot, () -> ArmPositions.stow[2]).schedule();
            // }));
            put("Invert", new InstantCommand(
                () -> SmartDashboard.putNumber("InvertTime", System.currentTimeMillis())// () -> RobotState.getInstance().invertBack()
            ));
            put("Balance", new Balance(drive));
        }};
    }

    public HashMap<String, Command> getEventMap() {
        return eventMap;
    }*/