package frc.robot.commands.auto;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.drive.Drive;

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

        // To transfrom blue path to red path on the x-axis do 16.53-x
        // To rotate blue path to red path adjust rotation 180 and set the heading to (180-abs(heading))*sign(heading)
        // Load path group
        PathConstraints constraints = new PathConstraints(AutoConstants.kMaxVelocityMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
        List<PathPlannerTrajectory> pathGroup = pathName.contains("-1-") || pathName.contains("-3-") ? PathPlanner.loadPathGroup(pathName, constraints) : PathPlanner.loadPathGroup("B-1-1", constraints);

        /*
            0-0: nothing
            1-1: 2.5
            1-2: 2 + balance
            2-1: 1
            2-2: 1 + balance
            3-1: 2.5
            3-2: 2 + balance
        */

        if (!pathName.equals("0-0")) {
            addCommands(
                resetOdometry(pathGroup),
                new InstantCommand(intake::toggleIntake),
                home(),
                setInverted(true),
                placeConeInitial(),
                invert()
            );
        }
        if (pathName.endsWith("2-1")) {
            addCommands(
                stow(),
                hold()
            );
        }
        if (pathName.endsWith("2-2")) {
            addCommands(
                stow(),
                new InstantCommand(drive::resetHeading),
                new ParallelCommandGroup(
                    balance(),
                    hold()
                )
            );
        }
        if (pathName.contains("-1-") || pathName.contains("-3-")) {
            addCommands(
                new ParallelRaceGroup(
                    drive(pathGroup.get(0)),
                    pickupCube().andThen(hold())
                ),
                new ParallelRaceGroup(
                    drive(pathGroup.get(1)),
                    invert().andThen(preparePlaceCube()).andThen(hold())
                ),
                placeCube(),
                invert()
            );
        }
        if (pathName.endsWith("1-1") || pathName.endsWith("3-1")) {
            addCommands(
                new ParallelRaceGroup(
                    drive(pathGroup.get(2)),
                    pickupCone().andThen(hold())
                ),
                new ParallelRaceGroup(
                    drive(pathGroup.get(3)),
                    invert().andThen(preparePlaceCone()).andThen(hold())
                ),
                placeCone(),
                stow(),
                hold()
            );
        }
        if (pathName.endsWith("1-2") || pathName.endsWith("3-2")) {
            addCommands(
                new ParallelRaceGroup(
                    drive(pathGroup.get(2)),
                    pickupCone().andThen(hold())
                ),
                new ParallelRaceGroup(
                    drive(pathGroup.get(3)).andThen(new InstantCommand(drive::resetHeading)).andThen(balance()),
                    invert().andThen(stow()).andThen(hold())
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
            // moveArmSpecial(new double[]{0.9, 0.6, 0}),
            moveArm(ArmPositions.placeConeUpHigh),
            placeCone()
        );
    }

    private Command placeCone() {
        return new ParallelRaceGroup(
            hold(),
            new SequentialCommandGroup(
                new InstantCommand(intake::place),
                new WaitCommand(0.2),
                new InstantCommand(intake::stop)
            )
        );
    }

    private Command placeCube() {
        return new ParallelRaceGroup(
            hold(),
            new SequentialCommandGroup(
                new InstantCommand(intake::place),
                new WaitCommand(0.25),
                new InstantCommand(intake::stop)
            )
        );
    }

    private Command stow() {
        return moveArmSpecial(ArmPositions.stow);
    }

    private Command preparePlaceCube() {
        return moveArmSpecial(ArmPositions.placeCubeHigh);
    }

    private Command preparePlaceCone() {
        return moveArmSpecial(ArmPositions.placeConeUpHigh);
    }

    private Command pickupCone() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> RobotState.getInstance().setMode(GamePieceMode.ConeUp)),
            new InstantCommand(intake::toggleIntake),
            moveArmSpecial(ArmPositions.intakeConeUpFrontGround)
        );
    }

    private Command pickupCube() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> RobotState.getInstance().setMode(GamePieceMode.Cube)),
            new InstantCommand(intake::toggleIntake),
            moveArmSpecial(ArmPositions.intakeCubeGround)
        );
    }

    private Command hold() {
        return new ParallelCommandGroup(
            new HoldPivot(pivot, telescope),
            new HoldTelescope(telescope, pivot),
            new HoldWrist(wrist, pivot)
        );
    }

    private Command setInverted(boolean inverted) {
        return new InstantCommand(() -> RobotState.getInstance().setBack(inverted));
    }

    private Command invert() {
        return new InstantCommand(() -> RobotState.getInstance().invertBack());
    }

    private Command balance() {
        return new Balance(drive);
    }

    private Command home() {
        return new InstantCommand(() -> {
            telescope.resetOffset();
            telescope.homed = true;
            wrist.resetOffset();
            wrist.homed = true;
        }); 
    }

    private Command moveArm(double[] position) {
        return moveArm(position, false);
    }
    
    private Command moveArm(double[] position, boolean wristIgnoreValidation) {
        return new ParallelRaceGroup(
            new MovePivot(pivot, position[0]).andThen(new HoldPivot(pivot, telescope)),
            new MoveTelescope(telescope, pivot, position[1], position[0]).andThen(new HoldTelescope(telescope, pivot)),
            // new MoveWrist(wrist, pivot, position[2], wristIgnoreValidation).andThen(new HoldWrist(wrist, pivot)),
            new WaitUntilCommand(() -> (telescope.atGoal && pivot.atGoal && wrist.atGoal))
        );
    }
    
    private Command moveArmSpecial(double[] position) {
        return new SequentialCommandGroup(
            new ParallelRaceGroup(
                new HoldPivot(pivot, telescope),
                new MoveTelescope(telescope, pivot, 0.05, position[0], true),
                // new MoveWrist(wrist, pivot, Math.PI / 2, false).andThen(new HoldWrist(wrist, pivot)),
                new WaitUntilCommand(() -> telescope.getPositionM() < 0.1)
            ),
            new ParallelRaceGroup(
                new MovePivot(pivot, position[0], true).andThen(new HoldPivot(pivot, telescope)),
                new HoldTelescope(telescope, pivot),
                // new MoveWrist(wrist, pivot, position[2], true).andThen(new HoldWrist(wrist, pivot)),
                new WaitUntilCommand(() -> (pivot.atGoal && wrist.atGoal))
            ),
            new ParallelRaceGroup(
                new MovePivot(pivot, position[0], false).andThen(new HoldPivot(pivot, telescope)),
                new MoveTelescope(telescope, pivot, position[1], position[0], false).andThen(new HoldTelescope(telescope, pivot)),
                // new MoveWrist(wrist, pivot, position[2], false).andThen(new HoldWrist(wrist, pivot)),
                new WaitUntilCommand(() -> (telescope.atGoal && pivot.atGoal && wrist.atGoal))
            )
        );
    }

    private Command followPath(Drive drive, PathPlannerTrajectory trajectory) {
        // return new FollowTrajectory(drive, trajectory);

        return new PPSwerveControllerCommand(
            trajectory, 
            () -> RobotState.getInstance().getFieldToVehicle(),
            new PIDController(AutoConstants.autoTranslationXKp, AutoConstants.autoTranslationXKi, AutoConstants.autoTranslationXKd), 
            new PIDController(AutoConstants.autoTranslationYKp, AutoConstants.autoTranslationYKi, AutoConstants.autoTranslationYKd), 
            new PIDController(AutoConstants.autoRotationKp, AutoConstants.autoRotationKi, AutoConstants.autoRotationKd), 
            (speeds) -> drive.setGoalChassisSpeeds(speeds),
            drive
        ).andThen(() -> drive.setGoalChassisSpeeds(new ChassisSpeeds(0, 0, 0)));
    }

}