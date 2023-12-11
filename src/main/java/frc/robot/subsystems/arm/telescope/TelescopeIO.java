package frc.robot.subsystems.arm.telescope;

import org.littletonrobotics.junction.AutoLog;

public interface TelescopeIO {
    
    @AutoLog
    public static class TelescopeIOInputs {
        public double positionM = 0.0;
        public double velocityMS = 0.0;
        public double appliedVolts = 0.0;
        public double statorCurrent = 0.0;
    }

    public default void updateInputs(TelescopeIOInputsAutoLogged inputs) {}

    public default void setBrakeMode(boolean brake) {}

    public default void setOutput(double volts) {}

    public default void setSensorPosition(double position) {}
}
