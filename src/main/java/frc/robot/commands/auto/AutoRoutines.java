package frc.robot.commands.auto;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Pose2d;
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
import frc.robot.commands.wrist.HomeWrist;
import frc.robot.commands.wrist.MoveWrist;
import frc.robot.commands.wrist.MoveWristAbsolute;
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

        /*
            1-1: 3
            1-2: 2.5 + balance
            1-3: 4
            2-1: 2 + balance left
            2-2: 2 + balance right
            3-1: 3
            3-2: 2.5 + balance
        */

        // To transfrom blue path to red path on the x-axis do 16.53-x
        // To rotate blue path to red path adjust rotation 180 and set the heading to (180-abs(heading))*sign(heading)
        // Load path group
        PathConstraints constraints = new PathConstraints(AutoConstants.maxVel, AutoConstants.maxAccel);
        if (pathName.endsWith("1-3")) {
            constraints = new PathConstraints(AutoConstants.maxVelFast, AutoConstants.maxAccelFast);
        }
        else if (pathName.contains("-2-")) {
            constraints = new PathConstraints(AutoConstants.maxVelSlow, AutoConstants.maxAccelSlow);
        }
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(pathName, constraints);

        if (!pathName.endsWith("1-3")) {
            addCommands(
                new InstantCommand(pivot::startAutoTimer),
                resetOdometry(pathGroup),
                home(),
                placeCone()
            );
        }
        if (pathName.contains("-2-")) {
            addCommands(
                new ParallelRaceGroup(
                    drive(pathGroup.get(0), true),
                    holdStow()
                ),
                new ParallelRaceGroup(
                    new WaitCommand(1).andThen(drive(pathGroup.get(1), true)),
                    intakeCube()
                ),
                new ParallelCommandGroup(
                    balance(false),
                    holdStow()
                )
            );
        }
        if ((pathName.contains("-1-") || pathName.contains("-3-")) && !pathName.endsWith("1-3")) {
            addCommands(
                new ParallelRaceGroup(
                    drive(pathGroup.get(0)),
                    intakeCube()
                ),
                new ParallelRaceGroup(
                    drive(pathGroup.get(1)),
                    preparePlaceCubeHigh()
                ),
                placeCube(),
                new ParallelRaceGroup(
                    drive(pathGroup.get(2)),
                    intakeCube()
                )
            );
        }
        if (pathName.endsWith("1-1") || pathName.endsWith("3-1")) {
            addCommands(
                new ParallelRaceGroup(
                    drive(pathGroup.get(3)),
                    preparePlaceCubeMid()
                ),
                placeCube(),
                new InstantCommand(pivot::stopAutoTimer),
                holdStow()
            );
        }
        if (pathName.endsWith("1-2") || pathName.endsWith("3-2")) {
            addCommands(
                new ParallelCommandGroup(
                    drive(pathGroup.get(3)).andThen(balance(false)),
                    holdStow()
                )
            );
        }
        if (pathName.endsWith("1-3")) {
            addCommands(
                new InstantCommand(pivot::startAutoTimer),
                resetOdometry(pathGroup),
                fakeHome(),
                spitCone(),
                new ParallelRaceGroup(
                    drive(pathGroup.get(0)),
                    intakeCube()
                ),
                new ParallelRaceGroup(
                    drive(pathGroup.get(1)),
                    preparePlaceCubeMid()
                ),
                placeCube(),
                new ParallelRaceGroup(
                    drive(pathGroup.get(2)),
                    intakeCube()
                ),
                new ParallelRaceGroup(
                    drive(pathGroup.get(3)),
                    preparePlaceCubeLow()
                ),
                placeCube(),
                new ParallelRaceGroup(
                    drive(pathGroup.get(4)),
                    intakeCube()
                ),
                new ParallelRaceGroup(
                    drive(pathGroup.get(5)),
                    preparePlaceCubeLow()
                ),
                placeCube(),
                new InstantCommand(pivot::stopAutoTimer),
                holdStow()
            );
        }
    }

    private Command resetOdometry(List<PathPlannerTrajectory> pathGroup) {
        return new InstantCommand( () -> {
            PathPlannerState initialState = pathGroup.get(0).getInitialState();
            drive.setHeading(0);
            drive.setFieldToVehicle(new Pose2d(initialState.poseMeters.getTranslation(), initialState.holonomicRotation));
        });
    }

    private Command drive(PathPlannerTrajectory trajectory) {
        return new FollowTrajectory(drive, trajectory, false);
    }

    private Command drive(PathPlannerTrajectory trajectory, boolean slow) {
        return new FollowTrajectory(drive, trajectory, slow);
    }

    private Command fakeHome() {
        return new InstantCommand(() -> {
            telescope.resetOffset();
            telescope.homed = true;
            wrist.resetOffset();
            wrist.homed = true;
        });
    }

    private Command home() {
        return new ParallelCommandGroup(
            new HomeTelescope(telescope),
            new HomeWrist(wrist)
        );
    }
    
    private Command placeCone() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> RobotState.getInstance().setMode(GamePieceMode.ConeUp)),
            new InstantCommand(() -> intake.setIntake(true)),
            new InstantCommand(() -> RobotState.getInstance().setBack(true)),
            new ParallelRaceGroup(
                new MovePivot(pivot, ArmPositions.placeConeUpHigh[0], false).andThen(new HoldPivot(pivot, telescope)),
                new MoveTelescope(telescope, pivot, ArmPositions.placeConeUpHigh[1], false).andThen(new HoldTelescope(telescope, pivot)),
                new MoveWrist(wrist, pivot, ArmPositions.placeConeUpHigh[2], false).andThen(new HoldWrist(wrist, pivot)),
                new WaitUntilCommand(() -> (pivot.atGoal && telescope.atGoal && wrist.atGoal))
            ),
            new ParallelRaceGroup(
                hold(),
                new SequentialCommandGroup(
                    new InstantCommand(intake::place),
                    new WaitCommand(0.2),
                    new InstantCommand(intake::stop)
                )
            )
        );
    }
    
    private Command intakeCube() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> RobotState.getInstance().setMode(GamePieceMode.Cube)),
            new InstantCommand(() -> RobotState.getInstance().setBack(false)),
            new ParallelRaceGroup(
                new HoldPivot(pivot, telescope),
                new MoveTelescope(telescope, pivot, 0.05, true),
                new HoldWrist(wrist, pivot),
                new WaitUntilCommand(() -> telescope.getPositionM() < 0.15)
            ),
            new InstantCommand(() -> intake.setIntake(true)),
            new ParallelCommandGroup(
                new MovePivot(pivot, ArmPositions.intakeCubeGround[0], false).andThen(new HoldPivot(pivot, telescope)),
                new MoveTelescope(telescope, pivot, ArmPositions.intakeCubeGround[1], false).andThen(new HoldTelescope(telescope, pivot)),
                new MoveWrist(wrist, pivot, ArmPositions.intakeCubeGround[2], false).andThen(new HoldWrist(wrist, pivot))
            )
        );
    }

    private Command preparePlaceCubeLow() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> RobotState.getInstance().setBack(true)),
            new ParallelRaceGroup(
                new MovePivot(pivot, ArmPositions.stow[0], true).andThen(new HoldPivot(pivot, telescope)),
                new MoveTelescope(telescope, pivot, ArmPositions.stow[1], true).andThen(new HoldTelescope(telescope, pivot)),
                new MoveWrist(wrist, pivot, ArmPositions.stow[2], true).andThen(new HoldWrist(wrist, pivot)),
                new WaitUntilCommand(() -> (pivot.atGoal && telescope.atGoal && wrist.atGoal))
            ),
            new ParallelCommandGroup(
                new MovePivot(pivot, ArmPositions.placeCubeLow[0], false).andThen(new HoldPivot(pivot, telescope)),
                new MoveTelescope(telescope, pivot, ArmPositions.placeCubeLow[1], false).andThen(new HoldTelescope(telescope, pivot)),
                new MoveWrist(wrist, pivot, ArmPositions.placeCubeLow[2], false).andThen(new HoldWrist(wrist, pivot))
            )
        );
    }

    private Command preparePlaceCubeMid() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> RobotState.getInstance().setBack(true)),
            new ParallelRaceGroup(
                new MovePivot(pivot, ArmPositions.stow[0], true).andThen(new HoldPivot(pivot, telescope)),
                new MoveTelescope(telescope, pivot, ArmPositions.stow[1], true).andThen(new HoldTelescope(telescope, pivot)),
                new MoveWrist(wrist, pivot, ArmPositions.stow[2], true).andThen(new HoldWrist(wrist, pivot)),
                new WaitUntilCommand(() -> (pivot.atGoal && telescope.atGoal && wrist.atGoal))
            ),
            new ParallelCommandGroup(
                new MovePivot(pivot, ArmPositions.placeCubeMid[0], false).andThen(new HoldPivot(pivot, telescope)),
                new MoveTelescope(telescope, pivot, ArmPositions.placeCubeMid[1], false).andThen(new HoldTelescope(telescope, pivot)),
                new MoveWrist(wrist, pivot, ArmPositions.placeCubeMid[2], false).andThen(new HoldWrist(wrist, pivot))
            )
        );
    }
    
    private Command preparePlaceCubeHigh() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> RobotState.getInstance().setBack(true)),
            new ParallelRaceGroup(
                new MovePivot(pivot, ArmPositions.stow[0], true).andThen(new HoldPivot(pivot, telescope)),
                new MoveTelescope(telescope, pivot, ArmPositions.stow[1], true).andThen(new HoldTelescope(telescope, pivot)),
                new MoveWrist(wrist, pivot, ArmPositions.stow[2], true).andThen(new HoldWrist(wrist, pivot)),
                new WaitUntilCommand(() -> (pivot.atGoal && telescope.atGoal && wrist.atGoal))
            ),
            new ParallelCommandGroup(
                new MovePivot(pivot, ArmPositions.placeCubeHigh[0], false).andThen(new HoldPivot(pivot, telescope)),
                new MoveTelescope(telescope, pivot, ArmPositions.placeCubeHigh[1], false).andThen(new HoldTelescope(telescope, pivot)),
                new MoveWrist(wrist, pivot, ArmPositions.placeCubeHigh[2], false).andThen(new HoldWrist(wrist, pivot))
            )
        );
    }

    private Command placeCube() {
        return new ParallelRaceGroup(   
            hold(),
            new SequentialCommandGroup(
                new InstantCommand(intake::shoot),
                new WaitCommand(0.1),
                new InstantCommand(intake::stop)
            )
        );
    }

    private Command flingCube() {
        return new SequentialCommandGroup(
            new ParallelRaceGroup(
                new WaitCommand(3),
                holdStow()
            ),
            new InstantCommand(() -> RobotState.getInstance().setBack(false)),
            new ParallelRaceGroup(
                new MovePivot(pivot, ArmPositions.preFlingCube[0], true).andThen(new HoldPivot(pivot, telescope)),
                new MoveTelescope(telescope, pivot, ArmPositions.preFlingCube[1], true).andThen(new HoldTelescope(telescope, pivot)),
                new MoveWrist(wrist, pivot, ArmPositions.preFlingCube[2], true).andThen(new HoldWrist(wrist, pivot)),
                new WaitUntilCommand(() -> (pivot.atGoal && telescope.atGoal && wrist.atGoal))
            ),
            new ParallelCommandGroup(
                new ParallelCommandGroup(
                    new MovePivot(pivot, ArmPositions.postFlingCube[0], false).andThen(new HoldPivot(pivot, telescope)),
                    new MoveTelescope(telescope, pivot, ArmPositions.postFlingCube[1], false).andThen(new HoldTelescope(telescope, pivot)),
                    new MoveWristAbsolute(wrist, pivot, ArmPositions.postFlingCube[2], ArmPositions.postFlingCube[0], false).andThen(new HoldWrist(wrist, pivot)),
                    new WaitUntilCommand(() -> (pivot.atGoal && telescope.atGoal && wrist.atGoal))
                ),
                new SequentialCommandGroup(
                    new WaitCommand(0.4),
                    new InstantCommand(intake::shootBackwards),
                    new WaitCommand(0.4),
                    new InstantCommand(intake::stop)
                )
            )
        );
    }

    private Command hold() {
        return new ParallelCommandGroup(
            new HoldPivot(pivot, telescope),
            new HoldTelescope(telescope, pivot),
            new HoldWrist(wrist, pivot)
        );
    }

    private Command holdStow() {
        return new ParallelCommandGroup(
            new MoveTelescope(telescope, pivot, ArmPositions.stow[1], true).andThen(new HoldTelescope(telescope, pivot)),
            new SequentialCommandGroup(
                new WaitUntilCommand(() -> telescope.atGoal),
                new ParallelCommandGroup(
                    new MovePivot(pivot, ArmPositions.stow[0], false).andThen(new HoldPivot(pivot, telescope)),
                    new MoveWrist(wrist, pivot, ArmPositions.stow[2], false).andThen(new HoldWrist(wrist, pivot))
                )
            )
        );
    }

    private Command balance(boolean forwards) {
        return new Balance(drive, forwards);
    }

    private Command spitCone() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> RobotState.getInstance().setMode(GamePieceMode.ConeUp)),
            new InstantCommand(intake::shoot),
            new WaitCommand(0.1),
            new InstantCommand(intake::stop)
        );
    }

}