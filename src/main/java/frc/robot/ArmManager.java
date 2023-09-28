package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.GamePieceMode;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.WristConstants;

public class ArmManager {

    private static ArmManager instance;

    public static ArmManager getInstance() {
        if (instance == null)
            instance = new ArmManager();
        return instance;
    }
    

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

    private boolean isIntaking = false;
    public void invertBack() {
        atBack = !atBack;
    }

    public void setBack(boolean back) {
        atBack = back;
    }

    public void putPivotDisplay(double posRad) {
        pivotLigament.setAngle(Units.radiansToDegrees(posRad));
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

    // controls -> arm
    public boolean atBack() {
        return atBack;
    }
    public GamePieceMode getMode() {
        return gamePieceMode;
    }
    public void setMode(Constants.GamePieceMode mode) {
        gamePieceMode = mode;

        // String str = mode == gamePieceMode.ConeUp ? "up" : "normal";
        // SmartDashboard.putString("Mode", str);
    }

    // intake -> robotcontainer | homing
    public boolean hasIntaked() {
        return hasIntaked;
    }
    public void setIntaked(boolean i) {
        hasIntaked = i;
    }

    // intake -> leds
    public boolean isIntaking() {
        return isIntaking;
    }
    public void setIntaking(boolean i) {
        isIntaking = i;
    }
}
