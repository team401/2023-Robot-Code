package frc.robot.commands.telescope;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

public class HomeTelescope extends CommandBase{
    private TelescopeSubsystem telescope;

    private Timer timer;

    public HomeTelescope(TelescopeSubsystem telescope) {
        this.telescope = telescope;

        timer = new Timer();

        addRequirements(telescope);
    }

    public void initialize() {
        telescope.setVolts(-1);
        timer.start();
        timer.reset();
    }

    public void execute() {
        if (telescope.getAmps() < 25) {
            timer.reset();
        }
    }

    public boolean isFinished() {
        return timer.hasElapsed(0.1);
    }

    public void end(boolean interrupted) {
        if (!interrupted) {
            telescope.resetOffset();
            telescope.homed = true;
        }

        telescope.stop();
    }


}