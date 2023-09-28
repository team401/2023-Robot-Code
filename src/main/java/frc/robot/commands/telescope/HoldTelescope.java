package frc.robot.commands.telescope;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.telescope.TelescopeSubsystem;

public class HoldTelescope extends CommandBase {
    private TelescopeSubsystem telescope;
    private PivotSubsystem pivot;

    private State goalState;

    public HoldTelescope(TelescopeSubsystem telescope, PivotSubsystem pivot) {
        this.telescope = telescope;
        this.pivot = pivot;

        addRequirements(telescope);
    }
    

    @Override
    public void initialize() {
        goalState = new State(telescope.getDesiredSetpoint().position, 0);
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("Pivot/hold setpoint", goalState.position);

        double output = telescope.calculateControl(goalState, pivot.getPositionRad());

        telescope.setVolts(output);
    }
}