package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;
import frc.robot.Constants.AutoConstants;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;

public class FollowTrajectory extends CommandBase {

    private final Drive drive;
    private Pose2d latestFieldToVehicle;
    private final RobotState robotState;
    private final PathPlannerTrajectory trajectory;
    private final PIDController xController = new PIDController(AutoConstants.autoTranslationXKp, AutoConstants.autoTranslationXKi, AutoConstants.autoTranslationXKd);
    private final PIDController yController = new PIDController(AutoConstants.autoTranslationYKp, AutoConstants.autoTranslationYKi, AutoConstants.autoTranslationYKd);
    private final ProfiledPIDController thetaController = 
        new ProfiledPIDController(
            AutoConstants.autoRotationKp, AutoConstants.autoRotationKi, AutoConstants.autoRotationKd,
            new TrapezoidProfile.Constraints(Math.PI / 4, Math.PI / 4)
        );

    private final HolonomicDriveController controller;

    private final Timer timer = new Timer();

    private final boolean verifyRotation;
    private final Timer rotationTimer = new Timer();
    private boolean rotationTimerStarted = false;

    public FollowTrajectory(Drive drive, PathPlannerTrajectory trajectory, boolean verifyRotation) {
        
        this.drive = drive;
        this.robotState = RobotState.getInstance();
        this.trajectory = trajectory;
        this.verifyRotation = verifyRotation;

        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        this.controller  = new HolonomicDriveController(xController, yController, thetaController);

        addRequirements(drive);

    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();

        rotationTimer.reset();
        rotationTimer.stop();

        xController.reset();
        yController.reset();
        thetaController.reset(new State(robotState.getFieldToVehicle().getRotation().getRadians(), 0));

        rotationTimerStarted = false;

    }

    @Override
    public void execute() {

    latestFieldToVehicle = robotState.getFieldToVehicle();

        if (!rotationTimerStarted) {
        
            PathPlannerState desiredState = (PathPlannerState) trajectory.sample(timer.get());
    
            ChassisSpeeds adjustedSpeeds = new ChassisSpeeds();
                adjustedSpeeds = controller.calculate(
                    latestFieldToVehicle, desiredState, desiredState.holonomicRotation);

            drive.setGoalChassisSpeeds(adjustedSpeeds);

            // SmartDashboard.putNumber("OutputSpeedX", adjustedSpeeds.vxMetersPerSecond);

            // SmartDashboard.putNumber("DesiredX", desiredState.poseMeters.getX());
            // SmartDashboard.putNumber("ActualX", RobotState.getInstance().getFieldToVehicle().getX());
            // SmartDashboard.putNumber("DesiredY", desiredState.poseMeters.getY());
            // SmartDashboard.putNumber("ActualY", RobotState.getInstance().getFieldToVehicle().getY());
            // SmartDashboard.putNumber("DesiredTheta", desiredState.holonomicRotation.getRadians());
            // SmartDashboard.putNumber("ActualTheta", RobotState.getInstance().getFieldToVehicle().getRotation().getRadians());

            if (timer.hasElapsed(trajectory.getTotalTimeSeconds())) {
                rotationTimerStarted = true;
                rotationTimer.reset();
                rotationTimer.start();
            }

        }
        else {
            PathPlannerState desiredState = (PathPlannerState) trajectory.getEndState();
            ChassisSpeeds adjustedSpeeds = controller.calculate(
                latestFieldToVehicle, desiredState, desiredState.holonomicRotation);
            drive.setGoalChassisSpeeds(adjustedSpeeds);
        }
    
    }

    @Override
    public boolean isFinished() {
        return (verifyRotation && rotationTimer.hasElapsed(0.75)) || (!verifyRotation && timer.hasElapsed(trajectory.getTotalTimeSeconds()));
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        drive.setGoalChassisSpeeds(new ChassisSpeeds(0, 0, 0));
    }

}