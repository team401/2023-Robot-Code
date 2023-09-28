package frc.robot.subsystems.drive;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.CANDevices;


public class AngleIOPidgeon2 implements AngleIO {
    private final Pigeon2 pigeon = new Pigeon2(CANDevices.pigeonIMU, CANDevices.canivoreName);

    private double yawOffsetRad = 0;
    private double pitchOffsetRad = 0;
    private double rollOffsetRad = 0;

    @Override
    public void updateInputs(AngleIOInputs inputs) {
        inputs.isConnnected = pigeon.getLastError().equals(ErrorCode.OK);
        inputs.yawRad = Units.degreesToRadians(pigeon.getYaw()) + yawOffsetRad;
        inputs.pitchRad = Units.degreesToRadians(pigeon.getPitch()) + pitchOffsetRad;
        inputs.rollRad = Units.degreesToRadians(pigeon.getRoll()) + rollOffsetRad;
    }

    @Override
    public void resetYaw() {
        yawOffsetRad = Units.degreesToRadians(pigeon.getYaw());
    }

    @Override
    public void setYaw(double yawRad) {
        resetYaw();
        yawOffsetRad -= yawRad;
    }

    @Override
    public void resetPitch() {
        pitchOffsetRad = Units.degreesToRadians(pigeon.getPitch());
    }

    @Override
    public void resetRoll() {
        rollOffsetRad = Units.degreesToRadians(pigeon.getRoll());
    }
}
