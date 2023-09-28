package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
    @AutoLog
    public static class WristIOInputs {
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    public default void updateInputs(WristIOInputs inputs) {}

    public default void setVolts(double volts) {}

    public default void setBrakeMode(boolean braked) {}

    public default void setOffset(double offset) {}

    public default void setCurrentLimit(double currentLimit, double triggerThresholdCurrent, double triggerThresholdTime) {}
}
