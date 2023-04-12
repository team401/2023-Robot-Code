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
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;

public class FollowTrajectory extends CommandBase {

    private final Drive drive;
    private Pose2d latestFieldToVehicle;
    private PathPlannerTrajectory trajectory;
    private PIDController xController;
    private PIDController yController;
    private ProfiledPIDController thetaController;

    private HolonomicDriveController controller;

    private final Timer timer = new Timer();

    public FollowTrajectory(Drive drive, PathPlannerTrajectory trajectory) {
        
        this.drive = drive;
        this.trajectory = trajectory;

        addRequirements(drive);

    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        
        xController = new PIDController(AutoConstants.autoTranslationXKp, AutoConstants.autoTranslationXKi, AutoConstants.autoTranslationXKd);
        yController = new PIDController(AutoConstants.autoTranslationYKp, AutoConstants.autoTranslationYKi, AutoConstants.autoTranslationYKd);
        thetaController = 
            new ProfiledPIDController(
                AutoConstants.autoRotationKp, AutoConstants.autoRotationKi, AutoConstants.autoRotationKd,
                new TrapezoidProfile.Constraints(Math.PI / 4, Math.PI / 4)
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
        
        RobotState.getInstance().setSimPose(new Pose2d(desiredState.poseMeters.getTranslation(), desiredState.holonomicRotation));

        // SmartDashboard.putNumber("DesiredX", desiredState.poseMeters.getX());
        // SmartDashboard.putNumber("DesiredY", desiredState.poseMeters.getY());
        // SmartDashboard.putNumber("DesiredOmega", desiredState.holonomicRotation.getRadians());
        // SmartDashboard.putNumber("ActualX", RobotState.getInstance().getFieldToVehicle().getX());
        // SmartDashboard.putNumber("ActualY", RobotState.getInstance().getFieldToVehicle().getY());
        // SmartDashboard.putNumber("ActualOmega", RobotState.getInstance().getFieldToVehicle().getRotation().getRadians());

        // SmartDashboard.putNumber("DesiredSpeedX", adjustedSpeeds.vxMetersPerSecond);
        // SmartDashboard.putNumber("ActualSpeedX", drive.getVelocity().vxMetersPerSecond);

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