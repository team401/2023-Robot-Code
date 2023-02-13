package frc.robot.commands.telescope;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TelescopeConstants;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

public class TestMoveTelescope extends CommandBase {
    private TelescopeSubsystem telescope;

    private State goalState;

    public TestMoveTelescope(TelescopeSubsystem telescope) {
        this.telescope = telescope;

        addRequirements(telescope);
    }
    

    @Override
    public void initialize() {
        goalState = new State(SmartDashboard.getNumber("Telescope test setpoint", 0), 0);
    }

    @Override
    public void execute() {

        double output = telescope.calculateControl(goalState, 0);

        // telescope.setVolts(output);
        telescope.setSimPos(goalState.position);
    }
}