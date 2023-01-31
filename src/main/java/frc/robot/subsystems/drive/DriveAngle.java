package frc.robot.subsystems.drive;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.CANDevices;

/**
 * Class used to interface with the gyro
 */
public class DriveAngle {

    /**
     * The gyroscope
     */
    private final Pigeon2 pigeon;

    /**
     * Offset used to the value of the heading
     */
    private double degHeadingOffset = 0;

    /**
     * Offset used to the value of the heading
     */
    private double degPitchOffset = 0;

    /**
     * Offset used to the value of the heading
     */
    private double degRollOffset = 0;

    /**
     * Initializes the gyro and offsets
     */
    public DriveAngle() {
        pigeon = new Pigeon2(CANDevices.pigeonIMU);
        degHeadingOffset = pigeon.getYaw();
        degPitchOffset = pigeon.getPitch();
        degRollOffset = pigeon.getRoll();
    }

    /**
     * @return the heading reported by the gyro minus the offset (radians)
     */
    public double getHeading() {
        return Units.degreesToRadians(pigeon.getYaw() - degHeadingOffset);
    }

    /**
     * Set the heading offset to make the gyro report a heading of zero
     */
    public void resetHeading() {
        degHeadingOffset = pigeon.getYaw();
    } 

    /**
     * @return the pitch reported by the gyro minus the offset (radians)
     */
    public double getPitch() {
        return Units.degreesToRadians(pigeon.getPitch() - degPitchOffset);
    }

    /**
     * Set the pitch offset to make the gyro report a heading of zero
     */
    public void resetPitch() {
        degPitchOffset = pigeon.getPitch();
    } 

    /**
     * @return the roll reported by the gyro minus the offset (radians)
     */
    public double getRoll() {
        return Units.degreesToRadians(pigeon.getRoll() - degRollOffset);
    }

    /**
     * Set the roll offset to make the gyro report a heading of zero
     */
    public void resetRoll() {
        degRollOffset = pigeon.getRoll();
    } 

    
}