package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ButtonMasher extends CommandXboxController {
    public ButtonMasher(int port) {
        super(port);
    }

    public Trigger leftStickLeft() {
        return new Trigger(() -> super.getLeftX() < -0.5);
    }

    public Trigger leftStickRight() {
        return new Trigger(() -> super.getLeftX() > 0.5);
    }

    public Trigger rightStickLeft() {
        return new Trigger(() -> super.getRightX() < -0.5);
    }

    public Trigger rightStickRight() {
        return new Trigger(() -> super.getRightX() > 0.5);
    }

    public Trigger rightStickUp() {
        return new Trigger(() -> super.getRightY() > 0.5);
    }

    public Trigger rightStickDown() {
        return new Trigger(() -> super.getRightX() < -0.5);
    }
}