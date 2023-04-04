package frc.robot.commands.wrist;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
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

        otherTimerStarted = false;

        wrist.setCurrentLimit(70, 80, 0.5);
    }

    public void execute() {
        // SmartDashboard.putNumber("wRISTaMPS", Math.abs(wrist.getAmps()));
        
        if (Math.abs(wrist.getAmps()) < 60) {
            timer.reset();
        }
        if (timer.hasElapsed(0.4) && !otherTimerStarted) {
            otherTimer.reset();
            otherTimer.start();
            otherTimerStarted = true;
            wrist.stop();
        }
    }

    public boolean isFinished() {
        return otherTimer.hasElapsed(0.4);
    }

    public void end(boolean interrupted) {
        if (!interrupted) {
            wrist.resetOffset();
            wrist.homed = true;
        }

        wrist.setCurrentLimit(50, 60, 0.5);
    
        // wrist.updateDesiredSetpointRad(
        //     new TrapezoidProfile.State(
        //         wrist.getPositionRad() + pivotPosRad.getAsDouble() - 0.3, 0));

        wrist.setVolts(0);
        wrist.updateDesiredSetpointRad(new TrapezoidProfile.State(Math.PI / 2, 0));
    }
}