package frc.robot.commands.auto;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
        // To rotate blue path to red path adjust rotation 180 and set the heading to (180-abs(heading))*sign(heading)
        // Load path group
        PathConstraints constraints = pathName.contains("-2-") ? 
            new PathConstraints(AutoConstants.kMaxVelocitySlowMetersPerSecond, AutoConstants.kMaxAccelerationSlowMetersPerSecondSquared) : 
            new PathConstraints(AutoConstants.kMaxVelocityMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(pathName, constraints);

        if (pathGroup.size() == 0) {
            SmartDashboard.putBoolean("FAILED TO LOAD PATH " + pathName, true);
            System.out.println("FAILED TO LOAD PATH " + pathName);
            return;
        }

        if (pathName.endsWith("1-1") || pathName.endsWith("3-1")) {
            addCommands(
                new InstantCommand(() -> {
                    PathPlannerState desiredState = (PathPlannerState) pathGroup.get(0).sample(0);
                    RobotState.getInstance().setSimPose(new Pose2d(desiredState.poseMeters.getTranslation(), desiredState.holonomicRotation));
                }),
                resetOdometry(pathGroup),
                 new InstantCommand(intake::toggleIntake),
                // home(),
                invert(),
                placeConeInitial(),
                new ParallelRaceGroup(
                    drive(pathGroup.get(0)),
                    invert().andThen(pickupCube()).andThen(hold())
                ),
                new ParallelRaceGroup(
                    drive(pathGroup.get(1)),
                    invert().andThen(moveArm(ArmPositions.stow)).andThen(preparePlaceCube()).andThen(hold())
                ),
                placeCube(),
                new ParallelRaceGroup(
                    drive(pathGroup.get(2)),
                    invert().andThen(pickupCone()).andThen(hold())
                ),
                new ParallelRaceGroup(
                    drive(pathGroup.get(3)),
                    invert().andThen(moveArm(ArmPositions.stow)).andThen(preparePlaceCone()).andThen(hold())
                ),
                placeCone(),
                new ParallelCommandGroup(
                    drive(pathGroup.get(4)).andThen(new InstantCommand(drive::resetHeading)),
                    invert().andThen(moveArm(ArmPositions.stow)).andThen(hold())
                )
            );
        }
        else if (pathName.endsWith("1-2") || pathName.endsWith("3-2")) {
            addCommands(
                new InstantCommand(() -> {
                    PathPlannerState desiredState = (PathPlannerState) pathGroup.get(0).sample(0);
                    RobotState.getInstance().setSimPose(new Pose2d(desiredState.poseMeters.getTranslation(), desiredState.holonomicRotation));
                }),
                resetOdometry(pathGroup),
                new InstantCommand(intake::toggleIntake),
                // home(),
                invert(),
                placeConeInitial(),
                new ParallelRaceGroup(
                    drive(pathGroup.get(0)),
                    invert().andThen(pickupCube()).andThen(hold())
                ),
                new ParallelRaceGroup(
                    drive(pathGroup.get(1)),
                    invert().andThen(moveArm(ArmPositions.stow)).andThen(preparePlaceCube()).andThen(hold())
                ),
                placeCube(),
                new ParallelRaceGroup(
                    drive(pathGroup.get(2)),
                    invert().andThen(pickupCone()).andThen(hold())
                ),
                new ParallelRaceGroup(
                    drive(pathGroup.get(3)),
                    invert().andThen(moveArm(ArmPositions.stow)).andThen(preparePlaceCone()).andThen(hold())
                ),
                placeCone(),
                new ParallelCommandGroup(
                    drive(pathGroup.get(4)).andThen(new InstantCommand(drive::resetHeading)).andThen(balance()),
                    invert().andThen(moveArm(ArmPositions.stow)).andThen(hold())
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

    private Command placeConeInitial() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> RobotState.getInstance().setMode(GamePieceMode.ConeUp)),
            new MoveWrist(wrist, pivot, ArmPositions.stow[2]).withTimeout(1),
            moveArm(ArmPositions.placeConeDownHigh),
            moveArm(ArmPositions.wristConePlaceHigh),
            new InstantCommand(intake::stop),
            moveArm(new double[]{ArmPositions.placeConeDownHigh[0], ArmPositions.placeConeDownHigh[1], ArmPositions.stow[2]}).withTimeout(0.3)
        );
    }

    private Command placeCone() {
        return new SequentialCommandGroup(
            moveArm(ArmPositions.wristConePlaceHigh),
            new InstantCommand(intake::stop),
            moveArm(new double[]{ArmPositions.placeConeDownHigh[0], ArmPositions.placeConeDownHigh[1], ArmPositions.stow[2]}).withTimeout(0.3)
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

    private Command preparePlaceCone() {
        return moveArm(ArmPositions.placeConeDownHigh);
    }

    private Command pickupCone() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> RobotState.getInstance().setMode(GamePieceMode.ConeDown)),
            new InstantCommand(intake::toggleIntake),
            moveArm(ArmPositions.intakeConeDownGround)
            // moveArm(ArmPositions.intakeConeDownExtendedGround)
        );
    }

    private Command pickupCube() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> RobotState.getInstance().setMode(GamePieceMode.Cube)),
            new InstantCommand(intake::toggleIntake),
            moveArm(ArmPositions.intakeCubeGround)
            // moveArm(ArmPositions.intakeCubeExtendedGround)
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
            new InstantCommand(() -> {wrist.resetOffset();wrist.homed = true;})
        );  
    }

    private Command moveArm(double[] position) {
        return new ParallelRaceGroup(
            new MovePivot(pivot, position[0]).andThen(new HoldPivot(pivot, telescope)),
            new MoveTelescope(telescope, pivot, position[1], position[0]).andThen(new HoldTelescope(telescope, pivot)),
            new MoveWrist(wrist, pivot, position[2]).andThen(new HoldWrist(wrist, pivot)),
            new WaitUntilCommand(() -> (telescope.atGoal && pivot.atGoal && wrist.atGoal))
        );
    }

    private Command followPath(Drive drive, PathPlannerTrajectory trajectory) {
        return new FollowTrajectory(drive, trajectory);
    }

}