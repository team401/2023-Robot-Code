package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GamePieceMode;
import frc.robot.Constants.LEDConstants;

public class LEDManager extends SubsystemBase {

    // LEDs
    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;

    // Rainbow
    private int rainbowFirstPixelHue = 0;

    // Other
    private final Timer flashTimer = new Timer();
    private boolean off = false;

    public LEDManager() {

        led = new AddressableLED(LEDConstants.ledPort);
        ledBuffer = new AddressableLEDBuffer(LEDConstants.baseLedCount + LEDConstants.armLedCount);
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();

        flashTimer.reset();
        flashTimer.start();

    }

    @Override
    public void periodic() {

        clear();
        
        if (!off) {
            if (DriverStation.isDisabled()){
                rainbow();
            }
            else {
                setSideIndicator();
                setIntakeModeIndicator();
                flashOnWhistle();
                flashActiveSide();
                flashOnIntake();
            }
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

    @SuppressWarnings("unused")
    private void epilepsy() {

        for (int i = 0; i < LEDConstants.armLedCount; i++) {
            setArmRGB(i, (int) (Math.random()*255), (int)(Math.random()*255), (int)(Math.random()*255)); 
        }

        for (int i = 0; i < LEDConstants.baseLedCount; i++) {
            setBaseRGB(i, (int)(Math.random()*255), (int)(Math.random()*255), (int)(Math.random()*255)); 
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
    }

    private void setIntakeModeIndicator() {
    }

    private void flashOnWhistle() {

        if (DriverStation.isTeleopEnabled() && Math.abs(DriverStation.getMatchTime()-30) < 1.5) {
            Color color = (flashTimer.get()*10%10)%5 < 2.5 ? Color.kBlack : LEDConstants.whistleFlashColor;
            for (int i = 0; i < LEDConstants.baseLedCount; i++) {
                setBaseLED(i, color);
            }
        }

    }

    private void flashOnIntake() {
    }

    public void setOff(boolean offf) {
        off = offf;
    }
    
}
