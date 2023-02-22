package frc.robot;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GamePieceMode;
import frc.robot.Constants.Position;
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

    private Mechanism2d displayMechanism = 
        new Mechanism2d(5, 5, new Color8Bit(Color.kWhite));
    private MechanismRoot2d root = displayMechanism.getRoot("arm", 2.5, 2.5);

    private MechanismLigament2d pivotLigament = root.append(
        new MechanismLigament2d(
            "pivot",
            0.4,
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
            0.2,
            0,
            3,
            new Color8Bit(Color.kCoral)));

    private GamePieceMode gamePieceMode = GamePieceMode.Cube;

    private final Field2d field = new Field2d();

    public void initializePoseEstimator(Rotation2d rotation, SwerveModulePosition[] modulePositions) {
        poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kinematics, rotation, modulePositions, new Pose2d(), 
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.08, 0.08, 0.08), // State measurement standard deviations. X, Y, theta.
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.02) // Vision measurement standard deviations. X, Y, theta.
            // Increase to trust less
        );
        SmartDashboard.putData(field);
    }

    public void recordDriveObservations(Rotation2d rotation, SwerveModulePosition[] modulePositions) {
        poseEstimator.updateWithTime(Timer.getFPGATimestamp(), rotation, modulePositions);
    }

    public void recordVisionObservations(Pose2d pose, double latencyS) {
        poseEstimator.addVisionMeasurement(pose, Timer.getFPGATimestamp()-latencyS);
    }

    public void setFieldToVehicle(Rotation2d rotation, SwerveModulePosition[] modulePositions, Pose2d fieldToVehicle) {
        poseEstimator.resetPosition(rotation, modulePositions, fieldToVehicle);
    }

    public Pose2d getFieldToVehicle() {
        SmartDashboard.putNumber("GetFieldToVehicleTime", System.currentTimeMillis());
        field.setRobotPose(poseEstimator.getEstimatedPosition());
        return poseEstimator.getEstimatedPosition();
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
    }

    public boolean hasIntaked() {
        return false;
    }

    public Command getMoveCommand(PivotSubsystem pivot, TelescopeSubsystem telescope, WristSubsystem wrist, Position position) {

        setStow(position.equals(Position.Stow));
        
        double[] positions = PositionHelper.getDouble(position, RobotState.getInstance().getMode());

        return new ParallelCommandGroup(
            new MovePivot(pivot,telescope,() -> positions[0]),
            new MoveTelescope(telescope, pivot, () -> positions[1],() -> positions[0]),
            new MoveWrist(wrist, pivot, () -> positions[2])
        );

    }
    
}
