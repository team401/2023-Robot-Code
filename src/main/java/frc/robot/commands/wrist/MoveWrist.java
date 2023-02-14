package frc.robot.commands.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotState;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class MoveWrist extends CommandBase {
    private WristSubsystem wrist;
    private PivotSubsystem pivot;

    private TrapezoidProfile profile;
    private State goalState;

    private double posRad;
    private double velRadS;

    private Timer timer;

    public MoveWrist(WristSubsystem wrist, PivotSubsystem pivot, double posRad, double velRadS) {
        this.wrist = wrist;
        this.pivot = pivot;
        this.posRad = posRad;
        this.velRadS = velRadS;

        addRequirements(this.wrist);
    }

    public MoveWrist(WristSubsystem wrist, PivotSubsystem pivot, double posRad) {
        this(wrist, pivot, posRad, 0);
    }

    @Override
    public void initialize() {

        goalState = new TrapezoidProfile.State(posRad, velRadS);

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

        // wrist.setVolts(output);
        wrist.setSimPosRad(setpoint.position - pivot.getPositionRad());
    }

    @Override
    public boolean isFinished() {
        return profile.isFinished(timer.get());
    }

    private double getAdjustedAngle() {
        return wrist.getPositionRad() + pivot.getPositionRad();
    }
}