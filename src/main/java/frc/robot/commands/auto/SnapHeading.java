package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.ArmManager;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.CustomHolonomicDrive;
import frc.robot.util.ExtendedPathPoint;

public class SnapHeading extends CommandBase {
    private final Drive drive;
    
    private final CustomHolonomicDrive holonomicDrive;
    
    private ExtendedPathPoint targetPose;
    
    public SnapHeading(Drive drive) {

        this.drive = drive;

        holonomicDrive = new CustomHolonomicDrive(
            new PIDController(0, 0, 0),
            new PIDController(DriveConstants.poseMoveRotationkP, 0, 0),
            new SlewRateLimiter(0),
            new SlewRateLimiter(0),
            new SlewRateLimiter(DriveConstants.poseMoveRotationMaxAccel)
        );

        addRequirements(drive);

    }

    @Override
    public void initialize() {

        Pose2d currentPose = drive.getFieldToVehicle();
        targetPose = new ExtendedPathPoint(new Translation2d(currentPose.getX(), currentPose.getY()), new Rotation2d(Math.abs(drive.getRotation().getRadians()) > Math.PI / 2 ? Math.PI - 0.01 : 0), new Rotation2d(Math.abs(drive.getRotation().getRadians()) > Math.PI / 2 ? Math.PI - 0.01 : 0));

    }

    @Override
    public void execute() {

        ChassisSpeeds speeds = holonomicDrive.calculate(drive.getFieldToVehicle(), targetPose.getPose2d());

        Transform2d distance = drive.getFieldToVehicle().minus(targetPose.getPose2d());
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
}
