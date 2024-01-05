package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GamePieceMode;

public class RobotState {

    private static RobotState instance;

    public static RobotState getInstance() {
        if (instance == null)
            instance = new RobotState();
        return instance;
    }
    
    private SwerveDrivePoseEstimator poseEstimator = 
        new SwerveDrivePoseEstimator(
            DriveConstants.kinematics,
            new Rotation2d(),
            new SwerveModulePosition[]{
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
            },
            new Pose2d());

    private boolean hasIntaked = false;
    
    private GamePieceMode gamePieceMode = GamePieceMode.ConeDown;

    private final Field2d mainField = new Field2d();
    private final Field2d targetField = new Field2d();
    private final Field2d odometryField = new Field2d();

    private SwerveDriveOdometry driveOdometry;

    private boolean isIntaking = false;

    public void initializePoseEstimator(Rotation2d rotation, SwerveModulePosition[] modulePositions) {
        poseEstimator = new SwerveDrivePoseEstimator(
            DriveConstants.kinematics,
            rotation,
            modulePositions,
            new Pose2d(),
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)) // doesn't matter
        );

        mainField.setRobotPose(new Pose2d(1.9, 4.99, Rotation2d.fromDegrees(0)));
        SmartDashboard.putData("Field Pose", mainField);
        SmartDashboard.putData("Target Pose", targetField);
        driveOdometry = new SwerveDriveOdometry(DriveConstants.kinematics, rotation, modulePositions);
    }

    public void recordDriveObservations(Rotation2d rotation, SwerveModulePosition[] modulePositions) {
        poseEstimator.update(rotation, modulePositions);
        driveOdometry.update(rotation, modulePositions);
    }

    public void recordVisionObservations(Pose2d pose, Matrix<N3, N1> stdDevs, double timestamp) {
        
        poseEstimator.addVisionMeasurement(pose, timestamp, stdDevs);
        mainField.setRobotPose(poseEstimator.getEstimatedPosition());

    }

    public void setFieldToVehicle(Rotation2d rotation, SwerveModulePosition[] modulePositions, Pose2d fieldToVehicle) {
        poseEstimator.resetPosition(rotation, modulePositions, fieldToVehicle);
        driveOdometry.resetPosition(rotation, modulePositions, fieldToVehicle);
    }

    public void setGoalPose(Pose2d pose) {
        targetField.setRobotPose(pose);
        SmartDashboard.putData("Target Pose", targetField);
    }

    public Pose2d getFieldToVehicle() {
        // SmartDashboard.putNumber("OdometryX", driveOdometry.getPoseMeters().getX());    
        // SmartDashboard.putNumber("OdometryY", driveOdometry.getPoseMeters().getY());
        // SmartDashboard.putNumber("OdometryTheta", driveOdometry.getPoseMeters().getRotation().getDegrees());

        mainField.setRobotPose(poseEstimator.getEstimatedPosition());
        SmartDashboard.putData("Field Pose", mainField);

        odometryField.setRobotPose(driveOdometry.getPoseMeters());
        SmartDashboard.putData("Odometry Pose", odometryField);
        
        return poseEstimator.getEstimatedPosition();    
    }

    public Pose2d getOdometryFieldToVehicle() {
        return driveOdometry.getPoseMeters();
    }

    public GamePieceMode getMode() {
        return gamePieceMode;
    }

    public void setMode(Constants.GamePieceMode mode) {
        gamePieceMode = mode;
        Logger.recordOutput("Arm/GamePieceMode", mode.name());
    }

    public boolean hasIntaked() {
        return hasIntaked;
    }

    public void setIntaked(boolean i) {
        hasIntaked = i;
    }

    public boolean isIntaking() {
        return isIntaking;
    }

    public void setIntaking(boolean i) {
        isIntaking = i;
    }
}
