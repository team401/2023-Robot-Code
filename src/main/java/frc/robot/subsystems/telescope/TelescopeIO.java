package frc.robot.subsystems.telescope;

import org.littletonrobotics.junction.AutoLog;

public interface TelescopeIO {
    @AutoLog
    public static class TelescopeIOInputs {
        public double positionMeters = 0.0;
        public double velocityMetersPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    public default void updateInputs(TelescopeIOInputs inputs) {}

    public default void setVolts(double volts) {}

    public default void setBrakeMode(boolean braked) {}

    public default void setOffset(double offset) {}

    public default void setCurrentLimit(double currentLimit, double triggerThresholdCurrent, double triggerThresholdTime) {}
}
