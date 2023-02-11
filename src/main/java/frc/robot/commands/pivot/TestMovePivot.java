package frc.robot.commands.pivot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

public class TestMovePivot extends CommandBase{
    
    private PivotSubsystem pivot;
    private TelescopeSubsystem telescope;
    
    private State goalState;
    
    public TestMovePivot(PivotSubsystem pivot) {
        this.pivot = pivot;

        addRequirements(this.pivot);
    }

    @Override
    public void initialize() {
        goalState = new State(
            SmartDashboard.getNumber("Pivot test setpoint", 0),
            0);
    }

    @Override
    public void execute() {
        // Calculate output from feedforward & PID
        double pivotOut = pivot.calculateControl(goalState, telescope.getPositionM());

        pivot.setVolts(pivotOut);
    }
}
