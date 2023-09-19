package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotState;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GamePieceMode;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.CustomHolonomicDrive;
import frc.robot.util.ExtendedPathPoint;

public class Align extends CommandBase {

    private final Drive drive;
    private final boolean left;
    
    private final CustomHolonomicDrive holonomicDrive;
    
    private ExtendedPathPoint targetPose;
    
    public Align(Drive drive, boolean left) {

        this.drive = drive;
        this.left = left;

        holonomicDrive = new CustomHolonomicDrive(
            new PIDController(DriveConstants.poseMoveTranslationkP, 0, 0),
            new PIDController(DriveConstants.poseMoveRotationkP, 0, 0),
            new SlewRateLimiter(DriveConstants.poseMoveTranslationMaxAccel),
            new SlewRateLimiter(DriveConstants.poseMoveTranslationMaxAccel),
            new SlewRateLimiter(DriveConstants.poseMoveRotationMaxAccel)
        );

        addRequirements(drive);

    }

    @Override
    public void initialize() {

        Pose2d goalPose = getGoalPose(left);
        SmartDashboard.putNumberArray("Target Pose", new Double[]{goalPose.getX(), goalPose.getY(), goalPose.getRotation().getDegrees()});
        targetPose = new ExtendedPathPoint(goalPose.getTranslation(), goalPose.getRotation(), goalPose.getRotation());

    }

    @Override
    public void execute() {

        ChassisSpeeds speeds = holonomicDrive.calculate(RobotState.getInstance().getFieldToVehicle(), targetPose.getPose2d());

        Transform2d distance = RobotState.getInstance().getFieldToVehicle().minus(targetPose.getPose2d());
        if (Math.abs(distance.getX()) < 0.01) speeds.vxMetersPerSecond = 0;
        if (Math.abs(distance.getY()) < 0.01) speeds.vyMetersPerSecond = 0;
        if (Math.abs(distance.getRotation().getRadians()) < 0.07) speeds.omegaRadiansPerSecond = 0;
        
        drive.setGoalChassisSpeeds(speeds);

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

        drive.setGoalChassisSpeeds(new ChassisSpeeds());

    }

    private Pose2d getGoalPose(boolean left) {

        boolean cube = RobotState.getInstance().getMode() == GamePieceMode.Cube;
        Pose2d currentPose = RobotState.getInstance().getFieldToVehicle();
        Rotation2d rotation = new Rotation2d(Math.abs(drive.getRotation().getRadians()) > Math.PI / 2 ? Math.PI - 0.01 : 0);

        if (DriverStation.getAlliance() == Alliance.Blue) {
            if (currentPose.getX() > 8)
                return left ? new Pose2d(15.5, 7.55, rotation) : new Pose2d(15.5, 5.95, rotation);
            double x = 1.95;
            double yCenter = currentPose.getY() > 3.58 ? 4.43 : currentPose.getY() > 1.91 ? 2.75 : 1.07;
            if (cube) return new Pose2d(x, yCenter, rotation);
            if (left) return new Pose2d(x, yCenter+0.56, rotation);
            return new Pose2d(x, yCenter-0.56, rotation);
        }
        if (DriverStation.getAlliance() == Alliance.Red) {
            if (currentPose.getX() < 8)
                return left ? new Pose2d(1, 5.95, rotation) : new Pose2d(1, 7.55, rotation);
            double x = 14.6;   
            double yCenter = currentPose.getY() > 3.58 ? 4.43 : currentPose.getY() > 1.91 ? 2.75 : 1.07;
            if (cube) return new Pose2d(x, yCenter, rotation);
            if (left) return new Pose2d(x, yCenter-0.56, rotation);
            return new Pose2d(x, yCenter+0.56, rotation);
        }
        return currentPose;

    }
    
}