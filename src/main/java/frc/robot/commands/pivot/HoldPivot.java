package frc.robot.commands.pivot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PivotSubsystem;
// import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

public class HoldPivot extends CommandBase {
    private PivotSubsystem pivot;
    private TelescopeSubsystem telescope;

    private State goalState;

    public double time;

    public HoldPivot(PivotSubsystem pivot, TelescopeSubsystem telescope) {
        this.pivot = pivot;
        this.telescope = telescope;

        addRequirements(this.pivot);
    }

    @Override
    public void initialize() {
        goalState = new State(pivot.getDesiredSetpointRad().position, 0);
    }

    @Override
    public void execute() {
        // Calculate output from feedforward & PID
        double pivotOut = pivot.calculateControl(goalState, telescope.getPositionM());

        pivot.setVolts(pivotOut);
   }
}
