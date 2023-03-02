package frc.robot.commands.auto;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotState;
import frc.robot.Constants.ArmPositions;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.GamePieceMode;
import frc.robot.commands.pivot.HoldPivot;
import frc.robot.commands.pivot.MovePivot;
import frc.robot.commands.telescope.HoldTelescope;
import frc.robot.commands.telescope.HomeTelescope;
import frc.robot.commands.telescope.MoveTelescope;
import frc.robot.commands.wrist.HoldWrist;
import frc.robot.commands.wrist.MoveWrist;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.drive.Drive;

public class AutoRoutines extends SequentialCommandGroup {

    private final Drive drive;
    private final PivotSubsystem pivot;
    private final TelescopeSubsystem telescope;
    private final WristSubsystem wrist;
    private final IntakeSubsystem intake;
    private final Vision vision;

    public AutoRoutines(String pathName, Drive driveSubsystem, PivotSubsystem pivotSubsystem, TelescopeSubsystem telescopeSubsystem, WristSubsystem wristSubsystem, IntakeSubsystem intakeSubsystem, Vision visionSubsystem) {

        drive = driveSubsystem;
        pivot = pivotSubsystem;
        telescope = telescopeSubsystem;
        wrist = wristSubsystem;
        intake = intakeSubsystem;
        vision = visionSubsystem;

        // To transfrom blue path to red path on the x-axis do 16.53-x
        // Load path group
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(pathName, new PathConstraints(AutoConstants.kMaxVelocityMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared));
        if (pathGroup.size() == 0) {
            SmartDashboard.putBoolean("FAILED TO LOAD PATH " + pathName, true);
            System.out.println("FAILED TO LOAD PATH " + pathName);
            return;
        }

        if (pathName.endsWith("1-1")) {
            addCommands(
                resetOdometry(pathGroup),
                home(),
                invert(),
                placeCone(),
                new ParallelRaceGroup(
                    drive(pathGroup.get(0)),
                    invert().andThen(pickupCube()).andThen(hold())
                ),
                new ParallelRaceGroup(
                    drive(pathGroup.get(1)),
                    invert().andThen(preparePlaceCube()).andThen(hold())
                ),
                placeCube(),
                invert(),
                moveArm(ArmPositions.stow),
                hold()
            );
        }
        else if (pathName.endsWith("1-2") || pathName.endsWith("3-2")) {
            addCommands(
                resetOdometry(pathGroup),
                home(),
                invert(),
                placeCone(),
                new ParallelRaceGroup(
                    drive(pathGroup.get(0)),
                    invert().andThen(pickupCube()).andThen(hold())
                ),
                new ParallelRaceGroup(
                    drive(pathGroup.get(1)),
                    invert().andThen(preparePlaceCube()).andThen(hold())
                ),
                placeCube(),
                new ParallelCommandGroup(
                    drive(pathGroup.get(2)).andThen(new InstantCommand(drive::resetHeading)).andThen(balance()),
                    invert().andThen(moveArm(ArmPositions.stow)).andThen(hold())
                )
            );
        }
        else if (pathName.endsWith("2-1")) {
            addCommands(
                resetOdometry(pathGroup),
                home(),
                new InstantCommand(drive::resetHeading),
                moveArm(ArmPositions.stow),
                hold()

            );
        }
        else if (pathName.endsWith("2-2")) {
            addCommands(
                resetOdometry(pathGroup),
                home(),
                new InstantCommand(drive::resetHeading),
                new ParallelCommandGroup(
                    balance(),
                    moveArm(ArmPositions.stow).andThen(hold())
                )
            );
        }
    }

    private Command resetOdometry(List<PathPlannerTrajectory> pathGroup) {
        return new InstantCommand( () -> {
            PathPlannerState initialState = pathGroup.get(0).getInitialState();
            drive.setHeading(initialState.holonomicRotation.getRadians());
            drive.setFieldToVehicle(new Pose2d(initialState.poseMeters.getTranslation(), initialState.holonomicRotation));
        });
    }

    private Command drive(PathPlannerTrajectory trajectory) {
        return new SequentialCommandGroup(
            followPath(drive, trajectory),
            new InstantCommand(() -> drive.setGoalChassisSpeeds(new ChassisSpeeds(0, 0, 0)))
        );
    }

    private Command center() {
        return new CenterTag(drive, vision);
    }

    private Command placeCone() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> RobotState.getInstance().setMode(GamePieceMode.ConeBack)),
            new InstantCommand(intake::toggleIntake),
            moveArm(ArmPositions.placeConeBackHigh),
            new InstantCommand(intake::stop),
            moveArm(ArmPositions.wristConePlaceHigh)
        );
    }

    private Command placeCube() {
        return new SequentialCommandGroup(
            new InstantCommand(intake::place),
            new WaitCommand(0.25),
            new InstantCommand(intake::stop)
        );
    }

    private Command preparePlaceCube() {
        return moveArm(ArmPositions.placeCubeHigh);
    }

    private Command pickupCube() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> RobotState.getInstance().setMode(GamePieceMode.Cube)),
            new InstantCommand(intake::toggleIntake),
            moveArm(ArmPositions.intakeCubeGround)
        );
    }

    private Command hold() {
        return new ParallelCommandGroup(
            new HoldPivot(pivot, telescope),
            new HoldTelescope(telescope, pivot),
            new HoldWrist(wrist, pivot)
        );
    }

    private Command invert() {
        return new InstantCommand(() -> RobotState.getInstance().invertBack());
    }

    private Command balance() {
        return new Balance(drive);
    }

    private Command home() {
        return new ParallelCommandGroup(
            new HomeTelescope(telescope),
            new InstantCommand(() -> { wrist.zeroOffset(); wrist.homed = true; })
        );
    }

    private Command moveArm(double[] position) {
        return new ParallelRaceGroup(
            new MovePivot(pivot, telescope, () -> position[0]).andThen(new HoldPivot(pivot, telescope)),
            new MoveTelescope(telescope, pivot, () -> position[1], () -> position[0]).andThen(new HoldTelescope(telescope, pivot)),
            new MoveWrist(wrist, pivot, () -> position[2]).andThen(new HoldWrist(wrist, pivot)),
            new WaitUntilCommand(() -> (telescope.atGoal && pivot.atGoal && wrist.atGoal))
        );
    }

    private Command followPath(Drive drive, PathPlannerTrajectory trajectory) {

        return new FollowTrajectory(drive, trajectory);
        // return new PPSwerveControllerCommand(
        //     trajectory, 
        //     () -> RobotState.getInstance().getFieldToVehicle(), // Pose supplier
        //     new PIDController(AutoConstants.autoTranslationKp, AutoConstants.autoTranslationKi, AutoConstants.autoTranslationKd), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
        //     new PIDController(AutoConstants.autoTranslationKp, AutoConstants.autoTranslationKi, AutoConstants.autoTranslationKd), // Y controller (usually the same values as X controller)
        //     new PIDController(AutoConstants.autoRotationKp, AutoConstants.autoRotationKi, AutoConstants.autoRotationKd), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
        //     drive::setGoalChassisSpeeds, // Module states consumer
        //     false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
        //     drive // Requires this drive subsystem
        // ).raceWith(new tmp(trajectory));

    }

}