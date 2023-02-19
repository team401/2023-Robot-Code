package frc.robot.oi;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface Driver {
    public default DoubleSupplier getDriveX() {
        return () -> 0.0;
    }

    public default DoubleSupplier getDriveY() {
        return () -> 0.0;
    }

    public default DoubleSupplier getRotation() {
        return () -> 0.0;
    }

    public default Trigger homeTelescope() {
        return new Trigger(() -> false);
    }

    public default Trigger homeWrist() {
        return new Trigger(() -> false);
    }
}
