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
    
    //Supplier for position of pivot
    private DoubleSupplier pivotPosRad;

    private Timer timer;

    public HomeWrist(WristSubsystem wrist, DoubleSupplier pivotPosRad) {
        this.wrist = wrist;

        timer = new Timer();

        addRequirements(wrist);
    
        this.pivotPosRad = pivotPosRad;
    }

    public void initialize() {

        wrist.setVolts(!wrist.atBack ? 4 : -4);

        timer.start();
        timer.reset();
    }

    public void execute() {
        if (Math.abs(wrist.getAmps()) < 20) {
            timer.reset();
        }
    }

    public boolean isFinished() {
        return timer.hasElapsed(0.1);
    }

    public void end(boolean interrupted) {
        if (!interrupted) {
            wrist.resetOffset();
            wrist.homed = true;
        }
    
        wrist.updateDesiredSetpointRad(
            new TrapezoidProfile.State(
                wrist.getPositionRad() + pivotPosRad.getAsDouble() - 0.3, 0));

        wrist.setVolts(0);
    }
}