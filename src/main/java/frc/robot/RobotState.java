package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GamePieceMode;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.WristConstants;

public class RobotState {

    private static RobotState instance;

    public static RobotState getInstance() {
        if (instance == null)
            instance = new RobotState();
        return instance;
    }
    
    private SwerveDrivePoseEstimator poseEstimator;

    private boolean atBack = false;

    private boolean atStow = true;

    private boolean hasIntaked = false;

    private Mechanism2d displayMechanism = 
        new Mechanism2d(5, 5, new Color8Bit(Color.kWhite));
    private MechanismRoot2d root = displayMechanism.getRoot("arm", 2.5, 0.43);

    private MechanismLigament2d pivotLigament = root.append(
        new MechanismLigament2d(
            "pivot",
            PivotConstants.lengthWOTeleM,
            0,
            4,
            new Color8Bit(Color.kPurple)));
    
    private MechanismLigament2d telescopeLigament = pivotLigament.append(
        new MechanismLigament2d(
            "telescope",
            0,
            0,
            3,
            new Color8Bit(Color.kBlue)));

    private MechanismLigament2d wrisLigament = telescopeLigament.append(
        new MechanismLigament2d(
            "wrist",
            WristConstants.intakeLengthM,
            0,
            3,
            new Color8Bit(Color.kCoral)));

    private GamePieceMode gamePieceMode = GamePieceMode.ConeDown;

    private final Field2d field = new Field2d();
    private final Field2d targetField = new Field2d();

    private SwerveDriveOdometry driveOdometry;

    private boolean isIntaking = false;

    public void initializePoseEstimator(Rotation2d rotation, SwerveModulePosition[] modulePositions) {
        poseEstimator = new SwerveDrivePoseEstimator(
            DriveConstants.kinematics,
            rotation,
            modulePositions,
            new Pose2d(),
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))
        );

        field.setRobotPose(new Pose2d(1.9, 4.99, Rotation2d.fromDegrees(0)));
        SmartDashboard.putData("Field Pose", field);
        SmartDashboard.putData("Target Pose", targetField);
        driveOdometry = new SwerveDriveOdometry(DriveConstants.kinematics, rotation, modulePositions);
    }

    public void recordDriveObservations(Rotation2d rotation, SwerveModulePosition[] modulePositions) {
        poseEstimator.update(rotation, modulePositions);
        driveOdometry.update(rotation, modulePositions);
    }

    public void recordVisionObservations(Pose2d pose, Matrix<N3, N1> stdDevs, double timestamp) {
        
        poseEstimator.addVisionMeasurement(pose, timestamp, stdDevs);
        field.setRobotPose(poseEstimator.getEstimatedPosition());

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

        field.setRobotPose(poseEstimator.getEstimatedPosition());
        SmartDashboard.putData("Field Pose", field);
        
        return poseEstimator.getEstimatedPosition();    
    }

    public Pose2d getOdometryFieldToVehicle() {
        return driveOdometry.getPoseMeters();
    }

    public void invertBack() {
        atBack = !atBack;
    }

    public void setBack(boolean back) {
        atBack = back;
    }


    public void putPivotDisplay(double posRad) {
        pivotLigament.setAngle(Units.radiansToDegrees(posRad));
        // SmartDashboard.putData("Arm Mechanism", displayMechanism);
    }

    public void putTelescopeDisplay(double posM) {
        telescopeLigament.setLength(posM);
        // SmartDashboard.putData("Arm Mechanism", displayMechanism);
    }

    public void putWristDisplay(double posRad) {
        wrisLigament.setAngle(Units.radiansToDegrees(posRad));
        // SmartDashboard.putData("Arm Mechanism", displayMechanism);
    }

    public void setStow(boolean stowed) {
        atStow = stowed;
    }

    public boolean atStow() {
        return atStow;
    }

    public boolean atBack() {
        return atBack;
    }

    public GamePieceMode getMode() {
        return gamePieceMode;
    }

    public void setMode(Constants.GamePieceMode mode) {
        gamePieceMode = mode;

        String str = mode == gamePieceMode.ConeUp ? "up" : "normal";
        // SmartDashboard.putString("Mode", str);
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
