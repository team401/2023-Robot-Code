package frc.robot.oi;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ThrustMasterDriver implements Driver {
    private Joystick leftJoystick;
    private Joystick rightJoystick;

    public ThrustMasterDriver(Joystick leftJoystick, Joystick rightJoystick) {
        this.leftJoystick = leftJoystick;
        this.rightJoystick = rightJoystick;
    }

    @Override
    public DoubleSupplier getDriveX() {
        return () -> -leftJoystick.getRawAxis(1);
    }

    @Override
    public DoubleSupplier getDriveY() {
        return () -> -leftJoystick.getRawAxis(0);
    }

    @Override
    public DoubleSupplier getRotation() {
        return () -> -rightJoystick.getRawAxis(0);
    }

    @Override
    public Trigger homeTelescope() {
        return new Trigger(() -> rightJoystick.getRawButton(5));
    }

    @Override
    public Trigger homeWrist() {
        return new Trigger(() -> leftJoystick.getRawButton(2));
    }
}
