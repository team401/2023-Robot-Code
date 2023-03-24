package frc.robot.commands.wrist;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
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

    private double posRad;

    private Timer timer = new Timer();

    public MoveWrist(WristSubsystem wrist, PivotSubsystem pivot, double posRad) {
        this.wrist = wrist;
        this.pivot = pivot;
        this.posRad = posRad;

        addRequirements(this.wrist);
    }

    @Override
    public void initialize() {

        goalState = new TrapezoidProfile.State(posRad, 0);

        if (RobotState.getInstance().atBack())
            goalState.position = Math.PI - goalState.position;

        timer.reset();
        timer.start();
        profile = new TrapezoidProfile(wrist.getConstraintsRad(), goalState,
            new State(getAdjustedAngle(), wrist.getVelRadS()));
        
        wrist.updateDesiredSetpointRad(goalState);

        wrist.resetPID();

        // SmartDashboard.putNumber("MoveWrist started", 1);

        wrist.atGoal = false;
    }

    @Override
    public void execute() {
        State setpoint = profile.calculate(timer.get());

        double output = wrist.calculateControl(setpoint, getAdjustedAngle(), false);

        // SmartDashboard.putNumber("Wrist Setpoint", setpoint.position);
        // SmartDashboard.putNumber("Wrist real pos", getAdjustedAngle());

        wrist.setVolts(output);
        wrist.setSimPosRad(setpoint.position - pivot.getPositionRad());
    }

    @Override
    public boolean isFinished() {
        return profile.isFinished(timer.get());
        // return false;
    }

    private double getAdjustedAngle() {
        return wrist.getPositionRad() + pivot.getPositionRad();
    }

    @Override
    public void end(boolean interrupted) {
        wrist.stop();

        wrist.atGoal = true;

        // SmartDashboard.putNumber("MoveWrist started", 0);
    }
}