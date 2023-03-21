package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotState;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GamePieceMode;
import frc.robot.subsystems.drive.Drive;

public class DriveToPose extends CommandBase {

    private final Drive drive;
    private Pose2d goalPose;

    private boolean left;

    private ProfiledPIDController xController = new ProfiledPIDController(DriveConstants.poseMoveTranslationkP, 0, 0, 
        new TrapezoidProfile.Constraints(DriveConstants.poseMoveTranslationMaxVel, DriveConstants.poseMoveTranslationMaxAccel));

    private ProfiledPIDController yController = new ProfiledPIDController(DriveConstants.poseMoveTranslationkP, 0, 0, 
        new TrapezoidProfile.Constraints(DriveConstants.poseMoveTranslationMaxVel, DriveConstants.poseMoveTranslationMaxAccel));

    private ProfiledPIDController thetaController = new ProfiledPIDController(DriveConstants.poseMoveRotationkP, 0, 0, 
        new TrapezoidProfile.Constraints(DriveConstants.poseMoveRotationMaxVel, DriveConstants.poseMoveRotationMaxAccel));

    public DriveToPose (Drive drive, boolean left) {
        this.drive = drive;

        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        
        addRequirements(drive);
    }

    @Override 
    public void initialize() {
        xController.reset(RobotState.getInstance().getFieldToVehicle().getX());
        yController.reset(RobotState.getInstance().getFieldToVehicle().getY());
        thetaController.reset(RobotState.getInstance().getFieldToVehicle().getRotation().getRadians());

        goalPose = getGoalPose();
    }

    @Override
    public void execute() {
        Pose2d currentPose = RobotState.getInstance().getFieldToVehicle();
        double xOutput = xController.calculate(currentPose.getX(), goalPose.getX());
        double yOutput = yController.calculate(currentPose.getY(), goalPose.getY());
        double thetaOutput = thetaController.calculate(currentPose.getRotation().getRadians(), goalPose.getRotation().getRadians());
        drive.setGoalChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(xOutput, yOutput, thetaOutput, drive.getRotation()));
    }
    
    @Override
    public void end(boolean interrupted) {
        drive.setGoalChassisSpeeds(new ChassisSpeeds(0,0,0));
    }

    private Pose2d getGoalPose() {

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
