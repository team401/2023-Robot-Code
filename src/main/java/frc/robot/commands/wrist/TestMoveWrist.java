package frc.robot.commands.wrist;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotState;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class TestMoveWrist extends CommandBase {
    private WristSubsystem wrist;
    private PivotSubsystem pivot;

    private Timer timer = new Timer();

    private State goalState;
    private TrapezoidProfile profile;

    public TestMoveWrist(WristSubsystem wrist, PivotSubsystem pivot) {
        this.wrist = wrist;
        this.pivot = pivot;

        addRequirements(this.wrist);
    }

    @Override
    public void initialize() {
        goalState = new State(
            1,
            0);

        if (RobotState.getInstance().atBack())
            goalState.position = Math.PI - goalState.position;

        timer.reset();
        timer.start();
        profile = new TrapezoidProfile(wrist.getConstraintsRad(), goalState,
            new State(getAdjustedAngle(), wrist.getVelRadS()));
        
        wrist.updateDesiredSetpointRad(goalState);

        wrist.resetPID();
    }

    @Override
    public void execute() {
        State setpoint = profile.calculate(timer.get());

        double output = wrist.calculateControl(setpoint, getAdjustedAngle());

        wrist.setVolts(output);
        // wrist.setSimPosRad(setpoint.position - pivot.getPositionRad());
    }

    @Override
    public boolean isFinished() {
        // return profile.isFinished(timer.get());
        return false;
    }

    private double getAdjustedAngle() {
        return wrist.getPositionRad() + pivot.getPositionRad();
    }
}