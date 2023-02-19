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
    private final AddressableLED armLed;
    private final AddressableLEDBuffer armLedBuffer;

    private final AddressableLED leftBaseLed;
    private final AddressableLEDBuffer leftBaseLedBuffer;

    private final AddressableLED rightBaseLed;
    private final AddressableLEDBuffer rightBaseLedBuffer;

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


    public LEDManager() {

        armLed = new AddressableLED(LEDConstants.armLedPort);
        armLedBuffer = new AddressableLEDBuffer(LEDConstants.armLedCount);
        armLed.setLength(armLedBuffer.getLength());
        armLed.setData(armLedBuffer);
        armLed.start();

        leftBaseLed = new AddressableLED(LEDConstants.leftBaseLedPort);
        leftBaseLedBuffer = new AddressableLEDBuffer(LEDConstants.baseLedCount);
        leftBaseLed.setLength(leftBaseLedBuffer.getLength());
        leftBaseLed.setData(leftBaseLedBuffer);
        leftBaseLed.start();

        rightBaseLed = new AddressableLED(LEDConstants.rightBaseLedPort);
        rightBaseLedBuffer = new AddressableLEDBuffer(LEDConstants.baseLedCount);
        rightBaseLed.setLength(rightBaseLedBuffer.getLength());
        rightBaseLed.setData(rightBaseLedBuffer);
        rightBaseLed.start();

        armClimbLedStates = new boolean[LEDConstants.armLedCount];
        baseClimbLedStates = new boolean[LEDConstants.baseLedCount/2];

        flashTimer.reset();
        flashTimer.start();

    }

    @Override
    public void periodic() {

        allianceColor = DriverStation.getAlliance() == DriverStation.Alliance.Red ? new Color(255, 0, 0) : new Color(0, 0, 255);
        
        if (!DriverStation.isDSAttached()) {
            preMatchClimbPattern();
        }
        else if (DriverStation.isDisabled()){
            rainbow();
        }
        else {
            setSideIndicator();
            flashActiveSide();
            setIntakeModeIndicator();
            flashOnWhistle();
            flashOnIntake();
        }

        armLed.setData(armLedBuffer);
        leftBaseLed.setData(leftBaseLedBuffer);
        rightBaseLed.setData(rightBaseLedBuffer);

    }

    private void rainbow() {
        for (int i = 0; i < LEDConstants.armLedCount; i++) {
            int hue = (rainbowFirstPixelHue + 90 + (i * 180 / LEDConstants.armLedCount)) % 180;
            armLedBuffer.setHSV(i, hue, 255, 128);
        }

        for (int i = 0; i < LEDConstants.baseLedCount; i++) {
            int hue = (rainbowFirstPixelHue + 90 + (i * 180 / LEDConstants.baseLedCount)) % 180;
            leftBaseLedBuffer.setHSV(i, hue, 255, 128);
            rightBaseLedBuffer.setHSV(i, hue, 255, 128);
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
            armLedBuffer.setLED(i, armClimbLedStates[i] ? allianceColor : Color.kBlack);
        }
        for (int i = 0; i < LEDConstants.baseLedCount/2; i++) {
            leftBaseLedBuffer.setLED(i, armClimbLedStates[i] ? allianceColor : Color.kBlack);
            rightBaseLedBuffer.setLED(i, armClimbLedStates[i] ? allianceColor : Color.kBlack);
            leftBaseLedBuffer.setLED(LEDConstants.baseLedCount-1-i, armClimbLedStates[i] ? allianceColor : Color.kBlack);
            rightBaseLedBuffer.setLED(LEDConstants.baseLedCount-1-i, armClimbLedStates[i] ? allianceColor : Color.kBlack);
        }

    }

    private void setSideIndicator() {

        for (int i = LEDConstants.baseLedCount/4; i < LEDConstants.baseLedCount/2; i++) {
            rightBaseLedBuffer.setRGB(i, 0, 255, 0);
            leftBaseLedBuffer.setRGB(i, 0, 255, 0);
        }
        for (int i = LEDConstants.baseLedCount/2; i < 3*LEDConstants.baseLedCount/4; i++) {
            rightBaseLedBuffer.setRGB(i, 255, 0, 0);
            leftBaseLedBuffer.setRGB(i, 255, 0, 0);
        }

    }

    private void flashActiveSide() {
        if (RobotState.getInstance().atStow() || (flashTimer.get()*10%10)%5 < 2.5) {
            return;
        }

        if (!RobotState.getInstance().atBack()) {
            for (int i = LEDConstants.baseLedCount/4; i < LEDConstants.baseLedCount/2; i++) {
                rightBaseLedBuffer.setLED(i, LEDConstants.activeSideFlashColor);
                leftBaseLedBuffer.setLED(i, LEDConstants.activeSideFlashColor);
            }
        }
        else {
            for (int i = LEDConstants.baseLedCount/2; i < 3*LEDConstants.baseLedCount/4; i++) {
                rightBaseLedBuffer.setLED(i, LEDConstants.activeSideFlashColor);
                leftBaseLedBuffer.setLED(i, LEDConstants.activeSideFlashColor);
            }
        }
    }

    private void setIntakeModeIndicator() {

        Color color = RobotState.getInstance().getMode() == GamePieceMode.ConeBack ? new Color(255, 255, 0) : new Color(255, 0, 255);
        for (int i = 0; i < LEDConstants.armLedCount; i++) {
            armLedBuffer.setLED(i, color);
        }
        for (int i = 0; i < LEDConstants.baseLedCount/4; i++) {
            rightBaseLedBuffer.setLED(i, color);
            leftBaseLedBuffer.setLED(i, color);
        }
        for (int i = 3*LEDConstants.baseLedCount/4; i < LEDConstants.baseLedCount; i++) {
            rightBaseLedBuffer.setLED(i, color);
            leftBaseLedBuffer.setLED(i, color);
        }

    }

    private void flashOnWhistle() {

        if (DriverStation.isTeleopEnabled() && Math.abs(DriverStation.getMatchTime()-30) < 2.5) {
            Color color = (flashTimer.get()*10%10)%4 < 2 ? Color.kBlack : LEDConstants.whistleFlashColor;
            for (int i = 0; i < LEDConstants.baseLedCount; i++) {
                rightBaseLedBuffer.setLED(i, color);
                leftBaseLedBuffer.setLED(i, color);
            }
        }

    }

    private void flashOnIntake() {

        if (RobotState.getInstance().hasIntaked()) {
            Color color = (flashTimer.get()*10%10)%5 < 2.5 ? Color.kBlack : LEDConstants.intakeFlashColor;
            for (int i = 0; i < LEDConstants.baseLedCount; i++) {
                rightBaseLedBuffer.setLED(i, color);
                leftBaseLedBuffer.setLED(i, color);
            }
        }

    }
    
}
