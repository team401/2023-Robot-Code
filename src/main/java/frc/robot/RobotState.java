package frc.robot;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GamePieceMode;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.Position;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.pivot.HoldPivot;
import frc.robot.commands.pivot.MovePivot;
import frc.robot.commands.telescope.MoveTelescope;
import frc.robot.commands.wrist.MoveWrist;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.util.PositionHelper;

public class RobotState {

    private static RobotState instance;

    public static RobotState getInstance() {
        if (instance == null)
            instance = new RobotState();
        return instance;
    }
    
    private SwerveDrivePoseEstimator poseEstimator;

    /**Whether the arm is supposed to be in the front or the back*/
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

    private SwerveDriveOdometry driveOdometry;

    private boolean isIntaking = false;

    public void initializePoseEstimator(Rotation2d rotation, SwerveModulePosition[] modulePositions) {
        poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kinematics, rotation, modulePositions, new Pose2d(), 
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.04, 0.04, 0.02), // State measurement standard deviations. X, Y, theta.
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 9999) // Vision measurement standard deviations. X, Y, theta.
            // Increase to trust less
        );
        field.setRobotPose(new Pose2d(1.9, 3.29, Rotation2d.fromDegrees(180)));
        SmartDashboard.putData(field);
        driveOdometry = new SwerveDriveOdometry(DriveConstants.kinematics, rotation, modulePositions);
    }

    public void recordDriveObservations(Rotation2d rotation, SwerveModulePosition[] modulePositions) {
        poseEstimator.updateWithTime(Timer.getFPGATimestamp(), rotation, modulePositions);
        driveOdometry.update(rotation, modulePositions);
    }

    public void recordVisionObservations(Pose2d pose, double latencyS) {
        poseEstimator.addVisionMeasurement(pose, Timer.getFPGATimestamp()-latencyS);
    }

    public void setFieldToVehicle(Rotation2d rotation, SwerveModulePosition[] modulePositions, Pose2d fieldToVehicle) {
        poseEstimator.resetPosition(rotation, modulePositions, fieldToVehicle);
        driveOdometry.resetPosition(rotation, modulePositions, fieldToVehicle);
    }

    public void setSimPose(Pose2d pose) {
        field.setRobotPose(pose);
    }

    public Pose2d getFieldToVehicle() {
        // field.setRobotPose(poseEstimator.getEstimatedPosition());
        // field.setRobotPose(driveOdometry.getPoseMeters());
        return poseEstimator.getEstimatedPosition();
    }

    public Pose2d getOdometryFieldToVehicle() {
        return driveOdometry.getPoseMeters();
    }

    public void invertBack() {
        atBack = !atBack;
    }

    public void putPivotDisplay(double posRad) {
        pivotLigament.setAngle(Units.radiansToDegrees(posRad));

        /*switch(getMode()) {
            case Cube:
                pivotLigament.setColor(new Color8Bit(Color.kPurple));
                break;
            default:
                pivotLigament.setColor(new Color8Bit(Color.kYellow));
            break;
        }*/

        SmartDashboard.putData("Arm Mechanism", displayMechanism);
    }

    public void putTelescopeDisplay(double posM) {
        telescopeLigament.setLength(posM);
        SmartDashboard.putData("Arm Mechanism", displayMechanism);
    }

    public void putWristDisplay(double posRad) {
        wrisLigament.setAngle(Units.radiansToDegrees(posRad));
        SmartDashboard.putData("Arm Mechanism", displayMechanism);
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
        SmartDashboard.putString("Mode", str);
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
