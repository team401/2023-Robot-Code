package frc.robot.commands.telescope;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TelescopeConstants;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

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

        double output = telescope.calculateControl(goalState, pivot.getPositionRad());

        telescope.setVolts(output);
    }
}