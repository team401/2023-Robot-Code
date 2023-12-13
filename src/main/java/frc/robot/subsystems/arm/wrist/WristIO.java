package frc.robot.subsystems.arm.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
    
    @AutoLog
    public static class WristIOInputs {
        public double positionRad = 0.0;
        public double velocityRadS = 0.0;
        public double appliedVolts = 0.0;
        public double statorCurrent = 0.0;
    }

    public default void updateInputs(WristIOInputsAutoLogged inputs) {}

    public default void setBrakeMode(boolean brake) {}

    public default void setOutput(double volts) {}
    
    public default void setSensorPosition(double position) {}
}
