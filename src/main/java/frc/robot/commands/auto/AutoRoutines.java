package frc.robot.commands.auto;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotState;
import frc.robot.Constants.ArmPositions;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.GamePieceMode;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem.ActiveArmSide;
import frc.robot.subsystems.arm.ArmSubsystem.ArmPosition;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.drive.Drive;

public class AutoRoutines extends SequentialCommandGroup {

    private final Drive drive;
    private final ArmSubsystem arm;
    private final IntakeSubsystem intake;

    private final Timer timer = new Timer();

    public AutoRoutines(
        String pathName,
        Drive driveSubsystem,
        ArmSubsystem armSubsystem,
        IntakeSubsystem intakeSubsystem
    ) {
        drive = driveSubsystem;
        arm = armSubsystem;
        intake = intakeSubsystem;

        timer.reset();

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

        if (!pathName.endsWith("1-3") && !pathName.contains("-2-")) {
            addCommands(
                new InstantCommand(timer::start),
                resetOdometry(pathGroup),
                placeCone()
                // new InstantCommand(pivot::autoConstrain)
            );
        }
        if (pathName.contains("-2-")) {
            addCommands(
                new InstantCommand(timer::start),
                resetOdometry(pathGroup),
                placeCubeInitial(),
                new ParallelRaceGroup(
                    drive(pathGroup.get(0), true).andThen(new Balance(drive)),
                    holdStow()
                )
            );
        }
        if (pathName.contains("1-1") || pathName.contains("1-2")) {
            addCommands(
                new ParallelRaceGroup(
                    new WaitCommand(0.7).andThen(drive(pathGroup.get(0))),
                    intakeCube()
                ),
                new ParallelRaceGroup(
                    drive(pathGroup.get(1)),
                    preparePlaceCubeHigh()
                ),
                placeCube(),
                new ParallelRaceGroup(
                    new WaitCommand(0.7).andThen(drive(pathGroup.get(2))),
                    intakeCube()
                )
            );
        }
        if (pathName.contains("3-1") || pathName.contains("3-2")) {
            addCommands(
                new ParallelRaceGroup(
                    new WaitCommand(0.7).andThen(drive(pathGroup.get(0))),
                    intakeCubeBump()
                ),
                new ParallelRaceGroup(
                    drive(pathGroup.get(1)),
                    preparePlaceCubeHigh()
                ),
                placeCube(),
                new ParallelRaceGroup(
                    new WaitCommand(0.7).andThen(drive(pathGroup.get(2))),
                    intakeCubeBump()
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
                new InstantCommand(timer::stop),
                holdStow()
            );
        }
        if (pathName.endsWith("1-2") || pathName.endsWith("3-2")) {
            addCommands(
                new ParallelCommandGroup(
                    holdStow(),
                    drive(pathGroup.get(3))
                        .andThen(new Balance(drive))
                )
            );
        }
        if (pathName.endsWith("1-3")) {
            addCommands(
                new InstantCommand(timer::start),
                resetOdometry(pathGroup),
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
                new InstantCommand(timer::stop),
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

    @SuppressWarnings("unused")
    private Command home() {
        return new InstantCommand(() -> {
            arm.homeWrist();
            arm.homeTelescope();
        });
    }
    
    private Command placeCone() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> RobotState.getInstance().setMode(GamePieceMode.ConeUp)),
            new InstantCommand(() -> intake.setIntake(true)),
            new InstantCommand(() -> arm.setArmSide(ActiveArmSide.FRONT)),
            arm.move(ArmPositions.placeConeUpHighPrepare, true),
            arm.move(ArmPositions.placeConeUpHighAuto, true),
            new SequentialCommandGroup(
                new InstantCommand(intake::place),
                new WaitCommand(0.3),
                new InstantCommand(intake::stop)
            )
        );
    }

    private Command placeCubeInitial() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> RobotState.getInstance().setMode(GamePieceMode.Cube)),
            new InstantCommand(() -> intake.setIntake(true)),
            new InstantCommand(() -> arm.setArmSide(ActiveArmSide.FRONT)),
            arm.move(ArmPositions.placeCubeHigh, true),
            new InstantCommand(intake::place),
            new WaitCommand(0.5),
            new InstantCommand(intake::stop)
        );
    }
    
    private Command intakeCube() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> RobotState.getInstance().setMode(GamePieceMode.Cube)),
            new InstantCommand(() -> arm.setArmSide(ActiveArmSide.BACK)),
            // FIXME cursed retraction implementation
            // We need a better API to implement odd auto optimizations
            arm.move(
                new ArmPosition(arm.getPosition().pivot(), 0.05, arm.getPosition().wrist()),
                false
            ).until(() -> arm.getPosition().telescope() < 0.25),
            new InstantCommand(() -> intake.setIntake(true)),
            arm.move(ArmPositions.intakeCubeGround, true)
        );
    }

    private Command intakeCubeBump() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> RobotState.getInstance().setMode(GamePieceMode.Cube)),
            new InstantCommand(() -> arm.setArmSide(ActiveArmSide.BACK)),
            // FIXME cursed retraction implementation
            // We need a better API to implement odd auto optimizations
            arm.move(
                new ArmPosition(arm.getPosition().pivot(), 0.05, arm.getPosition().wrist()),
                false).until(() -> arm.getPosition().telescope() < 0.25),
            new InstantCommand(() -> intake.setIntake(true)),
            arm.move(ArmPositions.intakeCubeGround, true)
        );
    }

    private Command preparePlaceCubeLow() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> arm.setArmSide(ActiveArmSide.FRONT)),
            arm.move(ArmPositions.stow, true),
            arm.move(ArmPositions.placeCubeLow, true)
        );
    }

    private Command preparePlaceCubeMid() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> arm.setArmSide(ActiveArmSide.FRONT)),
            arm.move(ArmPositions.stow, true),
            arm.move(ArmPositions.placeCubeHigh, true)
        );
    }
    
    private Command preparePlaceCubeHigh() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> arm.setArmSide(ActiveArmSide.BACK)),
            arm.move(ArmPositions.stow, true),
            arm.move(ArmPositions.placeCubeHigh, true)
        );
    }

    private Command placeCube() {
        return new ParallelRaceGroup(   
            new SequentialCommandGroup(
                new InstantCommand(intake::shoot),
                new WaitCommand(0.1),
                new InstantCommand(intake::stop)
            )
        );
    }

    private Command holdStow() {
        return arm.moveAndRetract(ArmPositions.stow, true);
    }

    private Command spitCone() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> RobotState.getInstance().setMode(GamePieceMode.ConeUp)),
            new InstantCommand(intake::shoot),
            new WaitCommand(0.2),
            new InstantCommand(intake::stop)
        );
    }

}