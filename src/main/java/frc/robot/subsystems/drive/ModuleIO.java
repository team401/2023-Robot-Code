package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    
    @AutoLog
    public static class ModuleIOInputs {
        public double drivePositionRad = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;

        public double rotationPositionRad = 0.0;
        public double rotationVelocityRadPerSec = 0.0;
        public double rotationAppliedVolts = 0.0;
        public double rotationCurrentAmps = 0.0;
    }

    public default void updateInputs(ModuleIOInputs inputs) {}

    public default void setDriveVoltage(double volts) {}

    public default void setRotationVoltage(double volts) {}

    public default void zeroEncoders() {}
    
    public default void toggleKill() {}
}
