package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.Constants.GamePieceMode;
import frc.robot.Constants.LEDConstants;

public class LEDManager extends SubsystemBase {

    // LEDs
    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;

    // Rainbow
    private int rainbowFirstPixelHue = 0;

    // Pre-Match Climb Pattern
    private final boolean[] armClimbLedStates;
    private final boolean[] baseClimbLedStates;
    private int armSpawnTime = 0;
    private int baseSpawnTime = 0;
    private int updateTime = 0;

    // Other
    private Color allianceColor = Color.kWhite;
    private final Timer flashTimer = new Timer();

    private boolean off = false;

    public LEDManager() {

        led = new AddressableLED(LEDConstants.ledPort);
        ledBuffer = new AddressableLEDBuffer(LEDConstants.baseLedCount + LEDConstants.armLedCount);
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();

        armClimbLedStates = new boolean[LEDConstants.armLedCount];
        baseClimbLedStates = new boolean[LEDConstants.baseLedCount/2];

        flashTimer.reset();
        flashTimer.start();

    }

    @Override
    public void periodic() {

        allianceColor = DriverStation.getAlliance() == DriverStation.Alliance.Red ? new Color(127, 0, 0) : new Color(0, 0, 127);

        if (!off) {
            if (!DriverStation.isDSAttached()) {
                // preMatchClimbPattern();
            }
            if (DriverStation.isDisabled()){
                rainbow();
            }
            else {
                clear();
                setSideIndicator();
                setIntakeModeIndicator();
                flashOnWhistle();
                flashActiveSide();
                flashOnIntake();
            }
        }
        else {
            clear();
        }

        led.setData(ledBuffer);

    }

    private void setArmRGB(int i, int r, int g, int b) {
        ledBuffer.setRGB(i+LEDConstants.baseLedCount, r, g, b);
    }

    private void setArmHSV(int i, int h, int s, int v) {
        ledBuffer.setHSV(i+LEDConstants.baseLedCount, h, s, v);
    }

    private void setArmLED(int i, Color color) {
        ledBuffer.setLED(i+LEDConstants.baseLedCount, color);
    }

    private void setBaseRGB(int i, int r, int g, int b) {
        ledBuffer.setRGB(i, r, g, b);
    }

    private void setBaseHSV(int i, int h, int s, int v) {
        ledBuffer.setHSV(i, h, s, v);
    }

    private void setBaseLED(int i, Color color) {
        ledBuffer.setLED(i, color);
    }

    private void clear() {
        for (int i = 0; i < LEDConstants.armLedCount; i++) { 
            setArmRGB(i, 0, 0, 0);
        }

        for (int i = 0; i < LEDConstants.baseLedCount; i++) {
            setBaseRGB(i, 0, 0, 0);
        }
    }

    private void rainbow() {
        for (int i = 0; i < LEDConstants.armLedCount; i++) {
            int hue = (rainbowFirstPixelHue + 90 + (i * 180 / LEDConstants.armLedCount)) % 180;
            setArmHSV(i, hue, 255, 127); 
        }

        for (int i = 0; i < LEDConstants.baseLedCount; i++) {
            int hue = (rainbowFirstPixelHue + 90 + (i * 180 / LEDConstants.baseLedCount)) % 180;
            setBaseHSV(i, hue, 255, 127);
        }
        
        if (LEDConstants.dynamicRainbow) {
            rainbowFirstPixelHue = (rainbowFirstPixelHue+LEDConstants.dynamicRainbowSpeed)%180;
        }

    }

    private void preMatchClimbPattern() {

        // Spawn
        armSpawnTime--;
        if (armSpawnTime <= 0) {
            int length = (int)(Math.random()*(LEDConstants.climbMaxLength-LEDConstants.climbMinLength)+LEDConstants.climbMinLength);
            armSpawnTime = (int)(Math.random()*(LEDConstants.climbMaxDelay-LEDConstants.climbMinDelay)+LEDConstants.climbMinDelay) + length*2;
            for (int i = 0; i < length && i < armClimbLedStates.length; i++)
                armClimbLedStates[i] = true;
        }
        baseSpawnTime--;
        if (baseSpawnTime <= 0) {
            int length = (int)(Math.random()*(LEDConstants.climbMaxLength-LEDConstants.climbMinLength)+LEDConstants.climbMinLength);
            baseSpawnTime = (int)(Math.random()*(LEDConstants.climbMaxDelay-LEDConstants.climbMinDelay)+LEDConstants.climbMinDelay) + length*2;
            for (int i = 0; i < length && i < baseClimbLedStates.length; i++)
                baseClimbLedStates[i] = true;
        }

        // Update
        updateTime--;
        if (updateTime <= 0) {
            updateTime = LEDConstants.climbSpeed;
            for (int i = armClimbLedStates.length-2; i > 0; i--)
                armClimbLedStates[i] = armClimbLedStates[i-1];
            armClimbLedStates[0] = false;
            for (int i = baseClimbLedStates.length-2; i > 0; i--)
                baseClimbLedStates[i] = baseClimbLedStates[i-1];
            baseClimbLedStates[0] = false;
        }

        // Buffer
        for (int i = 0; i < LEDConstants.armLedCount; i++) {
            setArmLED(i, armClimbLedStates[i] ? allianceColor : Color.kBlack);
        }
        for (int i = 0; i < LEDConstants.baseLedCount/2; i++) {
            setBaseLED(i, armClimbLedStates[i] ? allianceColor : Color.kBlack);
            setBaseLED(LEDConstants.baseLedCount-1-i, armClimbLedStates[i] ? allianceColor : Color.kBlack);
        }

    }

    private void setSideIndicator() {

        for (int i = LEDConstants.baseLedCount/2+1; i < 3*LEDConstants.baseLedCount/4; i++) {
            setBaseRGB(i, 0, 127, 0);
        }
        for (int i = LEDConstants.baseLedCount/4; i <= LEDConstants.baseLedCount/2; i++) {
            setBaseRGB(i, 127, 0, 0);
        }

    }

    private void flashActiveSide() {
        if ((flashTimer.get()*10%10)%5 < 3) {
            return;
        }

        if (!RobotState.getInstance().atBack()) {
            for (int i = LEDConstants.baseLedCount/2; i < 3*LEDConstants.baseLedCount/4; i++) {
                setBaseLED(i, LEDConstants.activeSideFlashColor);
            }
        }
        else {
            for (int i = LEDConstants.baseLedCount/4; i < LEDConstants.baseLedCount/2; i++) {
                setBaseLED(i, LEDConstants.activeSideFlashColor);
            }
        }
    }

    private void setIntakeModeIndicator() {

        Color color = RobotState.getInstance().getMode() != GamePieceMode.Cube ? new Color(127, 127, 0) : new Color(127, 0, 127);
        for (int i = 0; i < LEDConstants.baseLedCount/4; i++) {
            setBaseLED(i, color);
        }
        for (int i = 3*LEDConstants.baseLedCount/4; i < LEDConstants.baseLedCount; i++) {
            setBaseLED(i, color);
        }

        if (RobotState.getInstance().getMode() != GamePieceMode.ConeUp) {
            for (int i = 0; i < LEDConstants.armLedCount; i++) {
                setArmLED(i, color);
            }
        }
        else {
            Color yellow = new Color(127, 127, 0);
            Color black = new Color(0, 0, 0);
            for (int i = 0; i < LEDConstants.armLedCount; i++) {
                setArmLED(i, i/10%2 == 0 ? yellow : black);
            }
        }



    }

    private void flashOnWhistle() {

        if (DriverStation.isTeleopEnabled() && Math.abs(DriverStation.getMatchTime()-30) < 2.5) {
            Color color = (flashTimer.get()*10%10)%5 < 2.5 ? Color.kBlack : LEDConstants.whistleFlashColor;
            for (int i = 0; i < LEDConstants.baseLedCount; i++) {
                setBaseLED(i, color);
            }
        }

    }

    private void flashOnIntake() {
        if (RobotState.getInstance().isIntaking()) {
            Color color = (RobotState.getInstance().hasIntaked() && (flashTimer.get()*10%10)%5 < 2.5) ? Color.kBlack : LEDConstants.intakeFlashColor;
            for (int i = 0; i < LEDConstants.baseLedCount/4; i++) {
                setBaseLED(i, color);
            }
            for (int i = 3*LEDConstants.baseLedCount/4; i < LEDConstants.baseLedCount; i++) {
                setBaseLED(i, color);
            }
        }
    }

    public void setOff(boolean offf) {
        off = offf;
    }
    
}
