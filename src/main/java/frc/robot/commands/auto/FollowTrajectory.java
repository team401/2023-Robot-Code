package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotState;
import frc.robot.Constants.AutoConstants;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class FollowTrajectory extends Command {

    private final Drive drive;
    private Pose2d latestFieldToVehicle;
    private PathPlannerTrajectory trajectory;
    private PIDController xController;
    private PIDController yController;
    private ProfiledPIDController thetaController;

    private boolean slow;

    private HolonomicDriveController controller;

    private final Timer timer = new Timer();

    public FollowTrajectory(Drive drive, PathPlannerTrajectory trajectory, boolean slow) {
        
        this.drive = drive;
        this.trajectory = trajectory;
        this.slow = slow;

        addRequirements(drive);

    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        
        if (!slow) {
            xController = new PIDController(AutoConstants.autoTranslationXKp, AutoConstants.autoTranslationXKi, AutoConstants.autoTranslationXKd);
            yController = new PIDController(AutoConstants.autoTranslationYKp, AutoConstants.autoTranslationYKi, AutoConstants.autoTranslationYKd);
        }
        else {
            xController = new PIDController(AutoConstants.autoTranslationSlowXKp, AutoConstants.autoTranslationSlowXKi, AutoConstants.autoTranslationSlowXKd);
            yController = new PIDController(AutoConstants.autoTranslationSlowYKp, AutoConstants.autoTranslationSlowYKi, AutoConstants.autoTranslationSlowYKd);
        }
        thetaController = 
            new ProfiledPIDController(
                AutoConstants.autoRotationKp, AutoConstants.autoRotationKi, AutoConstants.autoRotationKd,
                new TrapezoidProfile.Constraints(Math.PI, Math.PI)
            );
        
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        controller  = new HolonomicDriveController(xController, yController, thetaController);
    }

    @Override
    public void execute() {

        latestFieldToVehicle = RobotState.getInstance().getFieldToVehicle();

        PathPlannerState desiredState = (PathPlannerState) trajectory.sample(timer.get());

        ChassisSpeeds adjustedSpeeds = new ChassisSpeeds();
            adjustedSpeeds = controller.calculate(
                latestFieldToVehicle, desiredState, desiredState.holonomicRotation);
        
        RobotState.getInstance().setGoalPose(new Pose2d(desiredState.poseMeters.getTranslation(), desiredState.holonomicRotation));

        drive.setGoalChassisSpeeds(adjustedSpeeds);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }

    @Override
    public void end(boolean interrupted) {
        drive.setGoalChassisSpeeds(new ChassisSpeeds(0, 0, 0));
    }

}