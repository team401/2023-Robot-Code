package frc.robot.commands.wrist;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotState;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class MoveWrist extends CommandBase {
    private WristSubsystem wrist;
    private PivotSubsystem pivot;

    private TrapezoidProfile profile;
    private State goalState;

    private double goalRad;
    private double pivotGoalRad;

    private Timer timer = new Timer();

    private Timer finishedTimer = new Timer();

    private final boolean ignoreValidation;

    public MoveWrist(WristSubsystem wrist, PivotSubsystem pivot, double goalRad, double pivotGoalRad) {
        this.wrist = wrist;
        this.pivot = pivot;
        this.goalRad = goalRad;
        this.pivotGoalRad = pivotGoalRad;
        this.ignoreValidation = false;

        addRequirements(this.wrist);
    }

    public MoveWrist(WristSubsystem wrist, PivotSubsystem pivot, double goalRad, double pivotGoalRad, boolean ignoreValidation) {
        this.wrist = wrist;
        this.pivot = pivot;
        this.goalRad = goalRad;
        this.pivotGoalRad = pivotGoalRad;
        this.ignoreValidation = ignoreValidation;

        addRequirements(wrist);
    }

    @Override
    public void initialize() {

        finishedTimer.reset();
        finishedTimer.start();

        goalRad -= pivotGoalRad;

        goalState = new TrapezoidProfile.State(goalRad, 0);

        if (RobotState.getInstance().atBack())
            goalState.position = Math.PI - goalState.position;


        timer.reset();
        timer.start();
        profile = new TrapezoidProfile(wrist.getConstraintsRad(), goalState,
            new State(wrist.getPositionRad(), wrist.getVelRadS()));
        
        wrist.updateDesiredSetpointRad(new TrapezoidProfile.State(goalRad+pivotGoalRad, 0));

        wrist.resetPID();

        SmartDashboard.putBoolean("MovingWrist", true);

        wrist.atGoal = false;
    }

    @Override
    public void execute() {
        State setpoint = profile.calculate(timer.get());

        double output = wrist.calculateControl(setpoint, wrist.getPositionRad(), false);

        SmartDashboard.putNumber("Wrist Setpoint", setpoint.position);
        SmartDashboard.putNumber("Wrist real pos", wrist.getPositionRad());

        wrist.setVolts(output);
        wrist.setSimPosRad(setpoint.position - pivot.getPositionRad());

        if (Math.abs(wrist.getPositionRad()-goalState.position) > Units.degreesToRadians(3)) {
            finishedTimer.reset();
        }
    }

    @Override
    public boolean isFinished() {
        return finishedTimer.hasElapsed(0.2) || (ignoreValidation && profile.isFinished(timer.get()));
        // return false;
    }

    private double getAdjustedAngle() {
        return wrist.getPositionRad() + pivot.getPositionRad();
    }

    @Override
    public void end(boolean interrupted) {
        wrist.stop();

        wrist.atGoal = true;

        SmartDashboard.putBoolean("MovingWrist", false);
    }
}