package frc.robot.commands.pivot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotState;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

public class TestMovePivot extends CommandBase{
    
    private PivotSubsystem pivot;
    private TelescopeSubsystem telescope;
    
    private TrapezoidProfile profile;
    private State goalState;

    public Timer timer = new Timer();
    
    public TestMovePivot(PivotSubsystem pivot, TelescopeSubsystem telescope) {
        this.pivot = pivot;
        this.telescope = telescope;

        addRequirements(this.pivot);
    }

    @Override
    public void initialize() {
        goalState = new State(
            SmartDashboard.getNumber("Pivot test setpoint", 0),
            0);

        if (RobotState.getInstance().atBack()) {
            goalState.position = Math.PI - goalState.position;
        }
        
        timer.reset();
        timer.start();
        profile = new TrapezoidProfile(pivot.getConstraintsRad(), goalState,
            new State(pivot.getPositionRad(), pivot.getVelRadS()));

        pivot.setDesiredSetpointRad(goalState);

        pivot.resetPID();
    }

    @Override
    public void execute() {
        State setpoint = profile.calculate(timer.get());

        // Calculate output from feedforward & PID
        double pivotOut = pivot.calculateControl(setpoint, telescope.getPositionM());

        // pivot.setVolts(pivotOut);
        pivot.setSimPos(setpoint.position);
    }

    @Override
    public boolean isFinished() {
        return profile.isFinished(timer.get());
    }
}
