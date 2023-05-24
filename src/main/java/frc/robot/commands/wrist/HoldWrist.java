package frc.robot.commands.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

public class HoldWrist extends CommandBase {
    private final WristSubsystem wrist;
    private final PivotSubsystem pivot;

    private final Timer timer = new Timer();
    private TrapezoidProfile profile;
    private State goalState;

    public HoldWrist(WristSubsystem wrist, PivotSubsystem pivot) {
        this.wrist = wrist;
        this.pivot = pivot;

        addRequirements(wrist);
    }

    @Override
    public void initialize() {
        goalState = new State(wrist.getDesiredSetpointRad().position, 0);
        // wrist.resetPID();

        timer.reset();
        timer.start();
        profile = new TrapezoidProfile(wrist.getConstraintsRad(), goalState,
            new State(getAdjustedAngle(), wrist.getVelRadS()));
        
    }

    @Override
    public void execute() {
        State setpoint = profile.calculate(timer.get());

        double output = wrist.calculateControl(setpoint, getAdjustedAngle(), true);

        wrist.setVolts(output);

        // SmartDashboard.putNumber("Wrist Setpoint", goalState.position);
        // SmartDashboard.putNumber("Wrist real pos", getAdjustedAngle());
    }

    private double getAdjustedAngle() {
        return wrist.getPositionRad() + pivot.getPositionRad();
    }
}