package frc.robot.commands.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotState;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GamePieceMode;
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

    private boolean left;

    public FollowTrajectory(Drive drive, boolean left) {
        this.drive = drive;
        this.left = left;

        addRequirements(drive);
    }

    public FollowTrajectory(Drive drive, PathPlannerTrajectory trajectory) {
        
        this.drive = drive;
        this.trajectory = trajectory;

        addRequirements(drive);

    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        
        if (DriverStation.isTeleop()) {
            Pose2d currentPose = RobotState.getInstance().getFieldToVehicle();
            Pose2d goalPose = getGoalPose(left);
            trajectory = PathPlanner.generatePath(
                new PathConstraints(DriveConstants.poseMoveTranslationMaxVel, DriveConstants.poseMoveTranslationMaxAccel),
                new PathPoint(currentPose.getTranslation(), new Rotation2d(0), currentPose.getRotation(), drive.getVelocity().vxMetersPerSecond),
                new PathPoint(goalPose.getTranslation(), new Rotation2d(0), goalPose.getRotation(), 0)
            );
            xController = new PIDController(DriveConstants.poseMoveTranslationkP, 0, 0);
            yController = new PIDController(DriveConstants.poseMoveTranslationkP, 0, 0);
            thetaController = 
                new ProfiledPIDController(
                    DriveConstants.poseMoveRotationkP, 0, 0,
                    new TrapezoidProfile.Constraints(Math.PI / 4, Math.PI / 4)
                );
        }
        else {
            xController = new PIDController(AutoConstants.autoTranslationXKp, AutoConstants.autoTranslationXKi, AutoConstants.autoTranslationXKd);
            yController = new PIDController(AutoConstants.autoTranslationYKp, AutoConstants.autoTranslationYKi, AutoConstants.autoTranslationYKd);
            thetaController = 
                new ProfiledPIDController(
                    AutoConstants.autoRotationKp, AutoConstants.autoRotationKi, AutoConstants.autoRotationKd,
                    new TrapezoidProfile.Constraints(Math.PI / 4, Math.PI / 4)
                );
        }
        
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
        
        // RobotState.getInstance().setSimPose(new Pose2d(desiredState.poseMeters.getTranslation(), desiredState.holonomicRotation));

        // adjustedSpeeds = new ChassisSpeeds(
        //     MathUtil.clamp(adjustedSpeeds.vxMetersPerSecond, -AutoConstants.kMaxVelocityMetersPerSecond, AutoConstants.kMaxVelocityMetersPerSecond), 
        //     MathUtil.clamp(adjustedSpeeds.vyMetersPerSecond, -AutoConstants.kMaxVelocityMetersPerSecond, AutoConstants.kMaxVelocityMetersPerSecond), 
        //     adjustedSpeeds.omegaRadiansPerSecond
        // );

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

    private Pose2d getGoalPose(boolean left) {

        boolean cube = RobotState.getInstance().getMode() == GamePieceMode.Cube;
        Pose2d currentPose = RobotState.getInstance().getFieldToVehicle();
        Rotation2d rotation = new Rotation2d(Math.abs(drive.getRotation().getRadians()) > Math.PI / 2 ? Math.PI - 0.01 : 0);

        if (DriverStation.getAlliance() == Alliance.Blue) {
            if (currentPose.getX() > 8)
                return left ? new Pose2d(15.5, 7.35, rotation) : new Pose2d(15.5, 6, rotation);
            double x = 1.9;
            double yCenter = currentPose.getY() > 3.58 ? 4.43 : currentPose.getY() > 1.91 ? 2.75 : 1.07;
            if (cube) return new Pose2d(x, yCenter, rotation);
            if (left) return new Pose2d(x, yCenter+0.56, rotation);
            return new Pose2d(x, yCenter-0.56, rotation);
        }
        if (DriverStation.getAlliance() == Alliance.Red) {
            if (currentPose.getX() < 8)
                return left ? new Pose2d(1, 6.13, rotation) : new Pose2d(1, 7.47, rotation);
            double x = 14.65;   
            double yCenter = currentPose.getY() > 3.58 ? 4.43 : currentPose.getY() > 1.91 ? 2.75 : 1.07;
            if (cube) return new Pose2d(x, yCenter, rotation);
            if (left) return new Pose2d(x, yCenter-0.56, rotation);
            return new Pose2d(x, yCenter+0.56, rotation);
        }
        return currentPose;

    }

}