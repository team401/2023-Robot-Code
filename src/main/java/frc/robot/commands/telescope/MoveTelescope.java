package frc.robot.commands.telescope;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.telescope.TelescopeSubsystem;

public class MoveTelescope extends CommandBase {
    private TelescopeSubsystem telescope;
    private PivotSubsystem pivot;

    private TrapezoidProfile profile;

    private double goalM;

    private Timer timer = new Timer();

    private Timer finishedTimer = new Timer();

    private final boolean ignoreValidation;

    public MoveTelescope(
        TelescopeSubsystem telescope,
        PivotSubsystem pivot,
        double goalM,
        boolean ignoreValidation) {
        
        this.goalM = goalM;
        this.telescope = telescope;
        this.pivot = pivot;
        this.ignoreValidation = ignoreValidation;
        
        addRequirements(telescope);
    }

    public MoveTelescope(TelescopeSubsystem telescope,
        PivotSubsystem pivot,
        double goalM) {
        this(telescope, pivot, goalM, false);
    }   

    @Override
    public void initialize() {
        timer.reset();
        timer.start();

        finishedTimer.reset();
        finishedTimer.start();

        profile = new TrapezoidProfile(telescope.getConstraints(), new State(goalM, 0), new State(telescope.getPositionM(), telescope.getVel()));

        telescope.setDesiredSetpoint(new State(goalM, 0));
        telescope.resetPID();

        telescope.atGoal = false;
    }

    @Override
    public void execute() {

        State setpoint = profile.calculate(timer.get());

        double output = telescope.calculateControl(setpoint, pivot.getPositionRad());

        // SmartDashboard.putNumber("Telescope/profiled setpoint", setpoint.position);
        // SmartDashboard.putNumber("Telescope/final setpoint", goalM);

        telescope.setVolts(output);

        if (Math.abs(telescope.getPositionM()-goalM) > 0.02) {
            finishedTimer.reset();
        }
    }

    @Override
    public boolean isFinished() {
        return finishedTimer.hasElapsed(0.2) || (ignoreValidation && profile.isFinished(timer.get()));
    }

    @Override
    public void end(boolean interrupted) {
        telescope.stop();
        telescope.atGoal = true;
    }
}
