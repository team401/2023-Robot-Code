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
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

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

        // Load path group"B-"+pathName
        List<PathPlannerTrajectory> bluePathGroup = PathPlanner.loadPathGroup("B-"+pathName, new PathConstraints(AutoConstants.kMaxVelocityMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared));
        List<PathPlannerTrajectory> redPathGroup = PathPlanner.loadPathGroup("R-"+pathName, new PathConstraints(AutoConstants.kMaxVelocityMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared));
        if (bluePathGroup.size() == 0 || redPathGroup.size() == 0) {
            SmartDashboard.putBoolean("FAILED TO LOAD PATH " + pathName, true);
            System.out.println("FAILED TO LOAD PATH " + pathName);
            return;
        }

        if (pathName.equals("1-1") || pathName.equals("3-1")) {
            addCommands(
                resetOdometry(bluePathGroup, redPathGroup),
                // home(),
                // invert(),
                // placeCone(),
                new ParallelRaceGroup(
                    drive(bluePathGroup.get(0), redPathGroup.get(0))
                    // invert().andThen(pickupCube()).andThen(hold())
                ),
                new ParallelRaceGroup(
                    drive(bluePathGroup.get(1), redPathGroup.get(1)).andThen(center()),
                    invert().andThen(preparePlaceCube()).andThen(hold())
                ),
                placeCube(),
                invert(),
                moveArm(ArmPositions.stow),
                hold()
            );
        }
        else if (pathName.equals("1-2") || pathName.equals("3-2")) {
            addCommands(
                resetOdometry(bluePathGroup, redPathGroup),
                home(),
                invert(),
                placeCone(),
                moveArm(ArmPositions.stow),
                new ParallelRaceGroup(
                    drive(bluePathGroup.get(0), redPathGroup.get(0)),
                    invert().andThen(pickupCube()).andThen(hold())
                ),
                new ParallelRaceGroup(
                    drive(bluePathGroup.get(1), redPathGroup.get(1)).andThen(center()),
                    invert().andThen(preparePlaceCube()).andThen(hold())
                ),
                placeCube(),
                new ParallelCommandGroup(
                    drive(bluePathGroup.get(2), redPathGroup.get(2)).andThen(balance()),
                    invert().andThen(moveArm(ArmPositions.stow)).andThen(hold())
                )
            );
        }
        else if (pathName.equals("2-1")) {
            addCommands(
                resetOdometry(bluePathGroup, redPathGroup),
                home(),
                moveArm(ArmPositions.stow),
                hold()

            );
        }
        else if (pathName.equals("2-2")) {
            addCommands(
                resetOdometry(bluePathGroup, redPathGroup),
                home(),
                new ParallelCommandGroup(
                    balance(),
                    moveArm(ArmPositions.stow).andThen(hold())
                )
            );
        }
    }

    private Command resetOdometry(List<PathPlannerTrajectory> bluePathGroup, List<PathPlannerTrajectory> redPathGroup) {
        return new InstantCommand( () -> {
            List<PathPlannerTrajectory> pathGroup = DriverStation.getAlliance().equals(Alliance.Blue) ? bluePathGroup : redPathGroup;
            PathPlannerState initialState = pathGroup.get(0).getInitialState();
            drive.setHeading(initialState.holonomicRotation.getRadians());
            drive.setFieldToVehicle(new Pose2d(initialState.poseMeters.getTranslation(), initialState.holonomicRotation));
        });
    }

    private Command drive(PathPlannerTrajectory blueTrajectory, PathPlannerTrajectory redTrajectory) {
        return new SequentialCommandGroup(
            followPath(drive, DriverStation.getAlliance().equals(Alliance.Blue) ? blueTrajectory : redTrajectory),
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
            moveArm(ArmPositions.wristConePlaceHigh),
            moveArm(new double[] {ArmPositions.placeConeBackHigh[0]+0.2, ArmPositions.placeConeBackHigh[1], ArmPositions.placeConeBackHigh[2]})
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

        return new PPSwerveControllerCommand(
            trajectory, 
            () -> RobotState.getInstance().getFieldToVehicle(), // Pose supplier
            new PIDController(AutoConstants.autoTranslationKp, AutoConstants.autoTranslationKi, AutoConstants.autoTranslationKd), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            new PIDController(AutoConstants.autoTranslationKp, AutoConstants.autoTranslationKi, AutoConstants.autoTranslationKd), // Y controller (usually the same values as X controller)
            new PIDController(AutoConstants.autoRotationKp, AutoConstants.autoRotationKi, AutoConstants.autoRotationKd), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            drive::setGoalChassisSpeeds, // Module states consumer
            false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            drive // Requires this drive subsystem
        );

    }

}