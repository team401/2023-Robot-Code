package frc.robot.commands.wrist;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
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

        addRequirements(this.wrist);
    }

    @Override
    public void initialize() {
        goalState = new State(wrist.getDesiredSetpointRad().position, 0);
    }

    @Override
    public void execute() {
        double output = wrist.calculateControl(goalState, getAdjustedAngle());

        // wrist.setVolts(output);
        wrist.setSimPosRad(goalState.position - pivot.getPositionRad());
    }

    private double getAdjustedAngle() {
        return wrist.getPositionRad() + pivot.getPositionRad();
    }
}