package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
    @AutoLog
    public static class PivotIOInputs {
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;

        public double appliedVoltsLeft = 0.0;
        public double currentAmpsLeft = 0.0;
        public double appliedVoltsRight = 0.0;
        public double currentAmpsRight = 0.0;

        public double appliedVoltsSim = 0.0;
        public double currentAmpsSim = 0.0;
    }

    public default void updateInputs(PivotIOInputs inputs) {}

    public default void setVolts(double volts) {}

    public default void setBrakeMode(boolean braked) {}
}
