package frc.robot.subsystems.arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    
    private Pivot pivot;
    private Telescope telescope;
    private Wrist wrist;

    public ActiveArmSide activeSide = ActiveArmSide.FRONT;

    public ArmSubsystem() {
        pivot = new Pivot(
            new TrapezoidProfile.Constraints(
                Units.degreesToRadians(360),
                Units.degreesToRadians(600)),
            () -> telescope.getPosition()
        );

        telescope = new Telescope(
            new TrapezoidProfile.Constraints(3, 3),
            () -> pivot.getPosition()
        );

        wrist = new Wrist(
            new TrapezoidProfile.Constraints(
                Units.degreesToRadians(630),
                Units.degreesToRadians(810)),
            () -> pivot.getPosition());
    }

    @Override
    public void periodic() {
        pivot.runControls();
        telescope.runControls();
        wrist.runControls();
    }

    public boolean atSetpoint() {
        return pivot.atSetpoint() && telescope.atSetpoint() && wrist.atSetpoint();
    }

    public ArmPosition getPosition() {
        return new ArmPosition(pivot.getPosition(), telescope.getPosition(), wrist.getPosition());
    }

    public void setSetpoint(ArmPosition setpoint) {
        if (activeSide == ActiveArmSide.FRONT) {
            pivot.setSetpoint(setpoint.pivot);
            telescope.setSetpoint(setpoint.telescope);
            wrist.setSetpoint(setpoint.wrist);
        }
        if (activeSide == ActiveArmSide.BACK) {
            pivot.setSetpoint(Math.PI - setpoint.pivot);
            telescope.setSetpoint(setpoint.telescope);
            wrist.setSetpoint(Math.PI - setpoint.wrist);
        }
    }

    public Command move(ArmPosition setpoint, boolean wait) {
        if (wait) {
            return new InstantCommand(() -> this.setSetpoint(setpoint))
                .andThen(Commands.waitUntil(this::atSetpoint));
        }

        return new InstantCommand(() -> this.setSetpoint(setpoint));
    }

    public static record ArmPosition(double pivot, double telescope, double wrist) { }

    public static enum ActiveArmSide {
        FRONT,
        BACK
    }
}
