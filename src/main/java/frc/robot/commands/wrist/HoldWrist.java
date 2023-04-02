package frc.robot.commands.wrist;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class HoldWrist extends CommandBase {
    private WristSubsystem wrist;
    private PivotSubsystem pivot;

    private State goalState;

    public HoldWrist(WristSubsystem wrist, PivotSubsystem pivot) {
        this.wrist = wrist;
        this.pivot = pivot;

        addRequirements(wrist);
    }

    @Override
    public void initialize() {
        goalState = new State(wrist.getDesiredSetpointRad().position, 0);
        wrist.resetPID();
    }

    @Override
    public void execute() {
        double output = wrist.calculateControl(goalState, getAdjustedAngle(), true);

        wrist.setVolts(output);
        wrist.setSimPosRad(goalState.position - pivot.getPositionRad());

        SmartDashboard.putNumber("Wrist Setpoint", goalState.position);
        SmartDashboard.putNumber("Wrist real pos", getAdjustedAngle());
    }

    private double getAdjustedAngle() {
        return wrist.getPositionRad() + pivot.getPositionRad();
    }
}