package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.drive.Drive;

public class FollowTrajectory extends CommandBase {

    private final Drive drive;
    private final Vision vision;
    private final PathPlannerTrajectory trajectory;

    private final PIDController xController = new PIDController(AutoConstants.autoTranslationKp, AutoConstants.autoTranslationKi, AutoConstants.autoTranslationKd);
    private final PIDController yController = new PIDController(AutoConstants.autoTranslationKp, AutoConstants.autoTranslationKi, AutoConstants.autoTranslationKd);
    private final ProfiledPIDController thetaController = 
        new ProfiledPIDController(
            AutoConstants.autoRotationKp, AutoConstants.autoRotationKi, AutoConstants.autoRotationKd, 
            new TrapezoidProfile.Constraints(DriveConstants.maxTurnRate, 4 * Math.PI)
        );

    private final HolonomicDriveController controller = new HolonomicDriveController(xController, yController, thetaController);

    private final Timer timer = new Timer();

    public FollowTrajectory(Drive driveSubsystem, Vision visionSubsystem, PathPlannerTrajectory trajectoryPath) {

        this.drive = driveSubsystem;
        this.vision = visionSubsystem;
        this.trajectory = trajectoryPath;

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drive);
        
    }

    @Override
    public void initialize() {

        timer.reset();
        timer.start();

    }

    @Override
    public void execute() {

        latestFieldToVehicle = robotState.getLatestFieldToVehicle();
        
        PathPlannerState desiredState = (PathPlannerState) trajectory.sample(timer.get());

        ChassisSpeeds adjustedSpeeds = new ChassisSpeeds();
        /*if (timer.get() / trajectory.getTotalTimeSeconds() >= 0.75 && vision.hasTarget()) {
            double omegaOut = xController.calculate(vision.getTX(), 0);
            desiredState.poseMeters = new Pose2d(0, desiredState.poseMeters.getY(), new Rotation2d(omegaOut));
            adjustedSpeeds = controller.calculate(
                latestFieldToVehicle, desiredState, desiredState.holonomicRotation);
        }
        else {*/
            adjustedSpeeds = controller.calculate(
                latestFieldToVehicle, desiredState, desiredState.holonomicRotation);
        //}

        drive.setGoalChassisSpeeds(adjustedSpeeds);

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean isInterrupted) {
        
    }
    
}
