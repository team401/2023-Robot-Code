package frc.robot.commands.wrist;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;

public class HomeWrist extends CommandBase {
    private WristSubsystem wrist;

    private Timer timer;
    private Timer otherTimer;
    private boolean otherTimerStarted = false;

    public HomeWrist(WristSubsystem wrist) {
        this.wrist = wrist;

        timer = new Timer();
        otherTimer = new Timer();

        addRequirements(wrist);
    }

    public void initialize() {

        wrist.setVolts(2);

        timer.start();
        timer.reset();

        otherTimer.reset();
        otherTimer.stop();
    }

    public void execute() {
        if (Math.abs(wrist.getAmps()) < 40) {
            timer.reset();
        }
        if (timer.hasElapsed(0.25) && !otherTimerStarted) {
            otherTimer.reset();
            otherTimer.start();
            otherTimerStarted = true;
            wrist.stop();
        }
    }

    public boolean isFinished() {
        return otherTimer.hasElapsed(0.3);
    }

    public void end(boolean interrupted) {
        if (!interrupted) {
            wrist.resetOffset();
            wrist.homed = true;
        }
    
        // wrist.updateDesiredSetpointRad(
        //     new TrapezoidProfile.State(
        //         wrist.getPositionRad() + pivotPosRad.getAsDouble() - 0.3, 0));

        wrist.setVolts(0);
        wrist.updateDesiredSetpointRad(new TrapezoidProfile.State(wrist.getPositionRad() - 0.05, 0));
    }
}