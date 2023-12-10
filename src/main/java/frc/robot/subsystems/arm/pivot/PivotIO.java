package frc.robot.subsystems.arm.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
    
    @AutoLog
    public static class PivotIOInputs {
        public double positionRad = 0.0;
        public double velocityRadS = 0.0;
        public double appliedVolts = 0.0;
        public double statorCurrent = 0.0;
    }

    public default void updateInputs(PivotIOInputsAutoLogged inputs) {}

    public default void setBrakeMode(boolean brake) {}

    public default void setOutput(double volts) {}
}
