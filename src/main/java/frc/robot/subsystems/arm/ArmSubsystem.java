package frc.robot.subsystems.arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TelescopeConstants;

public class ArmSubsystem extends SubsystemBase {

    private Pivot pivot;
    private Telescope telescope;
    private Wrist wrist;

    private boolean allActive = true;

    public ActiveArmSide activeSide = ActiveArmSide.FRONT;

    public ArmSubsystem() {
        pivot = new Pivot(
                new TrapezoidProfile.Constraints(
                        Units.degreesToRadians(360),
                        Units.degreesToRadians(600)),
                () -> telescope.getPosition());

        telescope = new Telescope(
                new TrapezoidProfile.Constraints(3, 3),
                () -> pivot.getPosition());

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

    public void homeWrist() {
        wrist.home();
    }

    public void homeTelescope() {
        telescope.home();
    }

    // The following methods are somewhat redundant; prone to be replaced
    public void togglePivotActive() {
        pivot.setActive(!pivot.isActive());
    }

    public void toggleTelescopeActive() {
        telescope.setActive(!telescope.isActive());
    }

    public void toggleWristActive() {
        wrist.setActive(!wrist.isActive());
    }

    public void jogPivotForward() {
        //Negative pivot values are closer to the front
        pivot.jogSetpointNegative();
    }

    public void jogPivotBackward() {
        //Negative pivot values are closer to the front
        pivot.jogSetpointPositive();
    }

    public void jogTelescopeOut() {
        telescope.jogSetpointPositive();
    }

    public void jogTelescopeIn() {
        telescope.jogSetpointNegative();
    }

    public void jogWristForward() {
        wrist.jogSetpointPositive();
    }

    public void jogWristBackward() {
        wrist.jogSetpointNegative();
    }

    public void toggleAllActive() {
        // this method forces all joints into the same active state; subject to change
        allActive = !allActive;

        pivot.setActive(allActive);
        telescope.setActive(allActive);
        wrist.setActive(allActive);
    }

    public void setBrakeMode(boolean brake) {
        pivot.setBrakeMode(brake);
        telescope.setBrakeMode(brake);
        wrist.setBrakeMode(brake);
    }

    public Command move(ArmPosition setpoint, boolean wait) {
        if (wait) {
            return new InstantCommand(() -> this.setSetpoint(setpoint))
                    .andThen(Commands.waitUntil(this::atSetpoint));
        }

        return new InstantCommand(() -> this.setSetpoint(setpoint));
    }

    public Command moveAndRetract(ArmPosition setpoint, boolean wait) {
        // TODO: implement telescope retraction more ergonomically. This probably
        // deserves a state machine of its own
        return Commands.sequence(
                // retract telescope
                new InstantCommand(() -> telescope.setSetpoint(TelescopeConstants.stowedPosition))
                        .andThen(Commands.waitUntil(this::atSetpoint)),
                // move pivot
                new InstantCommand(() -> pivot.setSetpoint(setpoint.pivot))
                        .andThen(Commands.waitUntil(this::atSetpoint)),
                // move to actual setpoint
                move(setpoint, wait));
    }

    public static record ArmPosition(double pivot, double telescope, double wrist) {
    }

    public static enum ActiveArmSide {
        FRONT,
        BACK
    }
}
