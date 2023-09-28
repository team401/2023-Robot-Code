package frc.robot.commands.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.telescope.TelescopeSubsystem;

/**
 * Command that holds the pivot at the position of its desiredSetpoint 
 * variable. Should be set as a default command for the pivot and only stopped
 * through interruption.
 */
public class HoldPivot extends CommandBase {
    private PivotSubsystem pivot;
    private TelescopeSubsystem telescope;

    private State goalState;
    private TrapezoidProfile profile;

    private final Timer timer = new Timer();

    public HoldPivot(PivotSubsystem pivot, TelescopeSubsystem telescope) {
        this.pivot = pivot;
        this.telescope = telescope;

        addRequirements(this.pivot);
    }

    @Override
    public void initialize() {
        goalState = new State(pivot.getDesiredSetpointRad().position, 0);

        profile = new TrapezoidProfile(pivot.getConstraintsRad(), goalState,
            new State(pivot.getPositionRad(), pivot.getVelRadS()));

        timer.reset();
        timer.start();

        pivot.setVolts(0);
    }

    @Override
    public void execute() {

        State setpoint = profile.calculate(timer.get());

        SmartDashboard.putNumber("Pivot/hold setpoint", setpoint.position);

        // Calculate output from feedforward & PID
        double pivotOut = MathUtil.clamp(pivot.calculateControl(setpoint, telescope.getPositionM()), -4, 4);

        pivot.setVolts(pivotOut);
   }
}
