package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface AngleIO {
    
    @AutoLog
    public static class AngleIOInputs {
        public double yawRad = 0.0;
        public double pitchRad = 0.0;
        public double rollRad = 0.0;
    }

    public default void updateInputs(AngleIOInputs inputs) {}

    public default void resetYaw() {}

    public default void setYaw(double yawRad) {}

    public default void resetPitch() {}

    public default void resetRoll() {}
}
