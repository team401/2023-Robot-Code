package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/**
 *  Represents the two motors controlling the wheels on the intake
 */
public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double leftOutputVolts = 0.0;
        public double rightOutputVolts = 0.0;

        public double leftCurrentAmps = 0.0;
        public double rightCurrentAmps = 0.0;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void setWheelVelocitiesVolts(double left, double right) {}
}
