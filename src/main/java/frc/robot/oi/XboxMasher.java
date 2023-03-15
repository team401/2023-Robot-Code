package frc.robot.oi;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class XboxMasher implements ButtonMasher{
    private XboxController controller;

    private boolean rumbling = false;
    private final Command endRumbleCommand = new InstantCommand(() -> {
		controller.setRumble(RumbleType.kBothRumble, 0);
		rumbling = false;
	});

    public XboxMasher(XboxController controller) {
        this.controller = controller;
    }

    @Override
    public Trigger killAll() {
        return new Trigger(() -> controller.getBackButton());
    }

    @Override
    public Trigger flipSide() {
        return new Trigger(() -> controller.getYButton())
            .onTrue(new InstantCommand(() -> {
                if (rumbling) {
                    controller.setRumble(RumbleType.kBothRumble, 0);
                } else {
                    controller.setRumble(RumbleType.kBothRumble, 1);
                }
                rumbling = !rumbling;
            }));
    }

    @Override
    public Trigger ground() {
        return new Trigger(() -> controller.getPOV() == 180)
            .onTrue(endRumbleCommand);
    }

    @Override
    public Trigger mid() {
        return new Trigger(() -> controller.getPOV() == 90)
            .onTrue(endRumbleCommand);
    }

    @Override
    public Trigger high() {
        return new Trigger(() -> controller.getPOV() == 0)
            .onTrue(endRumbleCommand);
    }

    @Override
    public Trigger shelf() {
        return new Trigger(() -> controller.getXButton())
            .onTrue(endRumbleCommand);
    }

    @Override
    public Trigger stow() {
        return new Trigger(() -> controller.getPOV() == 270)
            .onTrue(endRumbleCommand);
    }

    @Override
    public Trigger special() {
        return new Trigger(() -> controller.getAButton());
    }

    @Override
    public Trigger otherSpecial() {
        return new Trigger(() -> controller.getBButton());
    }

    @Override
    public Trigger cubeMode() {
        return new Trigger(() -> controller.getLeftBumper());
    }

    @Override
    public Trigger coneDownMode() {
        return new Trigger(() -> controller.getRightBumper());
    }

    public Trigger coneUpMode() {
        return new Trigger(() -> controller.getYButton());
    }

    @Override
    public  Trigger intake() {
        return new Trigger(() -> controller.getLeftTriggerAxis() > 0.3);
    }

    @Override
    public  Trigger place() {
        return new Trigger(() -> controller.getRightTriggerAxis() > 0.3);
    }

    @Override
    public  Trigger jogPivotUp() {
        return new Trigger(() -> controller.getLeftX() > 0.3);
    }

    @Override
    public Trigger jogPivotDown() {
        return new Trigger(() -> controller.getLeftX() < -0.3);
    }

    @Override
    public Trigger jogTelescopeUp() {
        return new Trigger(() -> controller.getRightX() > 0.3);
    }

    @Override
    public Trigger jogTelescopeDown() {
        return new Trigger(() -> controller.getRightX() < -0.3);
    }

    @Override
    public Trigger jogWristUp() {
        return new Trigger(() -> controller.getRightY() > 0.3);
    }

    @Override
    public Trigger jogWristDown() {
        return new Trigger(() -> controller.getRightY() < -0.3);
    }
}
