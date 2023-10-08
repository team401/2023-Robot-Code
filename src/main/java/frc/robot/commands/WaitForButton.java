package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class WaitForButton extends CommandBase {
    private CommandXboxController controller;

    public WaitForButton(CommandXboxController controller) {
        this.controller = controller;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return controller.a().getAsBoolean();
    }
}
