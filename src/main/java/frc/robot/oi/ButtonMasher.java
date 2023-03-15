package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.WristSubsystem;

public interface ButtonMasher {
    public default Trigger killAll() {
        return new Trigger(() -> false);
    }

    public default Trigger flipSide() {
        return new Trigger(() -> false);
    }

    public default Trigger ground() {
        return new Trigger(() -> false);
    }

    public default Trigger mid() {
        return new Trigger(() -> false);
    }

    public default Trigger high() {
        return new Trigger(() -> false);
    }

    public default Trigger shelf() {
        return new Trigger(() -> false);
    }

    public default Trigger stow() {
        return new Trigger(() -> false);
    }

    public default Trigger special() {
        return new Trigger(() -> false);
    }

    public default Trigger otherSpecial() {
        return new Trigger(() -> false);
    }

    public default Trigger cubeMode() {
        return new Trigger(() -> false);
    }

    public default Trigger coneMode() {
        return new Trigger(() -> false);
    }

    public default Trigger intake() {
        return new Trigger(() -> false);
    }

    public default Trigger place() {
        return new Trigger(() -> false);
    }

    public default Trigger jogPivotUp() {
        return new Trigger(() -> false);
    }

    public default Trigger jogPivotDown() {
        return new Trigger(() -> false);
    }

    public default Trigger jogTelescopeUp() {
        return new Trigger(() -> false);
    }

    public default Trigger jogTelescopeDown() {
        return new Trigger(() -> false);
    }

    public default Trigger jogWristUp() {
        return new Trigger(() -> false);
    }

    public default Trigger jogWristDown() {
        return new Trigger(() -> false);
    }
}
