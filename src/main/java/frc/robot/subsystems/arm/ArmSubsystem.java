package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.TelescopeConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.arm.pivot.Pivot;
import frc.robot.subsystems.arm.pivot.PivotIO;
import frc.robot.subsystems.arm.telescope.Telescope;
import frc.robot.subsystems.arm.telescope.TelescopeIO;
import frc.robot.subsystems.arm.wrist.Wrist;
import frc.robot.subsystems.arm.wrist.WristIO;

public class ArmSubsystem extends SubsystemBase {

    private Pivot pivot;
    private Telescope telescope;
    private Wrist wrist;

    private boolean allActive = true;

    private ActiveArmSide activeSide = ActiveArmSide.FRONT;

    private ArmMovementState state = ArmMovementState.DEFAULT;

    private ArmPosition setpoint = new ArmPosition(0, 0, 0);

    private Mechanism2d positionMechanism = 
        new Mechanism2d(5, 5, new Color8Bit(Color.kWhite));
    private MechanismRoot2d root = positionMechanism.getRoot("arm", 2.5, 0.43);

    private MechanismLigament2d pivotLigament = root.append(
        new MechanismLigament2d(
            "pivot",
            PivotConstants.lengthWOTeleM,
            0,
            4,
            new Color8Bit(Color.kPurple)));
    
    private MechanismLigament2d telescopeLigament = pivotLigament.append(
        new MechanismLigament2d(
            "telescope",
            0,
            0,
            3,
            new Color8Bit(Color.kBlue)));

    private MechanismLigament2d wristLigament = telescopeLigament.append(
        new MechanismLigament2d(
            "wrist",
            WristConstants.intakeLengthM,
            0,
            3,
            new Color8Bit(Color.kCoral)));


    public ArmSubsystem(PivotIO pivotIO, TelescopeIO telescopeIO, WristIO wristIO) {
        pivot = new Pivot(
                pivotIO,
                new TrapezoidProfile.Constraints(
                        Units.degreesToRadians(360),
                        Units.degreesToRadians(600)),
                () -> telescope.getPosition());

        telescope = new Telescope(
                telescopeIO,
                new TrapezoidProfile.Constraints(3, 3),
                () -> pivot.getPosition());

        wrist = new Wrist(
                wristIO,
                new TrapezoidProfile.Constraints(
                        Units.degreesToRadians(630),
                        Units.degreesToRadians(810)),
                () -> pivot.getPosition());

        // telescope.setActive(false);
        wrist.setActive(false);
        pivot.setActive(false);
    }

    @Override
    public void periodic() {
        switch (state) {
            case RETRACT_TELESCOPE:
                if (telescope.atSetpoint()) {
                    pivot.setSetpoint(setpoint.pivot);
                    state = ArmMovementState.MOVE_PIVOT;
                }
                break;
            case MOVE_PIVOT:
                if (pivot.atSetpoint()) {
                    telescope.setSetpoint(setpoint.telescope);
                    wrist.setSetpoint(setpoint.wrist);
                    state = ArmMovementState.DEFAULT;
                }
                break;
            case DEFAULT:
                break;
        }

        pivot.runControls();
        telescope.runControls();
        wrist.runControls();
        
        Logger.recordOutput("Arm/Side", activeSide.name());
        Logger.recordOutput("Arm/MovementState", state.name());

        pivotLigament.setAngle(Units.radiansToDegrees(pivot.getPosition()));
        telescopeLigament.setLength(telescope.getPosition());
        wristLigament.setAngle(Units.radiansToDegrees(wrist.getPosition()));

        Logger.recordOutput("Arm/Mechanism", positionMechanism);

        Logger.recordOutput("Arm/Pivot/Setpoint", pivot.setpoint);
        Logger.recordOutput("Arm/Telescope/Setpoint", telescope.setpoint);
        Logger.recordOutput("Arm/Wrist/Setpoint", wrist.setpoint);

        Logger.recordOutput("Arm/Pivot/Position", pivot.getPosition());
        Logger.recordOutput("Arm/Telescope/Position", telescope.getPosition());
        Logger.recordOutput("Arm/Wrist/Position", wrist.getPosition());

        SmartDashboard.putNumber("Arm/time", System.currentTimeMillis());
    }

    public boolean atSetpoint() {
        return pivot.atSetpoint() && telescope.atSetpoint() && wrist.atSetpoint();
    }

    public ArmPosition getPosition() {
        return new ArmPosition(pivot.getPosition(), telescope.getPosition(), wrist.getPosition());
    }

    /**
     * Signal this subsystem to move to the setpoint provided. If the setpoint is extremely different
     * from the current state of the arm, it will retract the telescope first.
     * @param setpoint
     */
    public void setSetpoint(ArmPosition setpoint) {
        if (activeSide == ActiveArmSide.FRONT) {
            this.setpoint = new ArmPosition(setpoint.pivot, setpoint.telescope, setpoint.wrist);
        }
        if (activeSide == ActiveArmSide.BACK) {
            this.setpoint = new ArmPosition(
                Math.PI - setpoint.pivot,
                setpoint.telescope,
                Math.PI - setpoint.wrist
            );
        }

        if (Math.abs(this.setpoint.pivot - this.getPosition().pivot) > 0.3) {
            telescope.setSetpoint(TelescopeConstants.stowedPosition);
            state = ArmMovementState.RETRACT_TELESCOPE;
        } else {
            pivot.setSetpoint(this.setpoint.pivot);
            telescope.setSetpoint(this.setpoint.telescope);
            wrist.setSetpoint(this.setpoint.wrist);
        }
    }

    /**
     * Signals the wrist to home. The wrist will make no attempt to reach a setpoint while it is homing.
     */
    public void homeWrist() {
        wrist.home();
    }

    /**
     * Signals the telescope to home. The telescope will make no attempt to reach a setpoint while
     * it is homing.
     */
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

    /**
     * Toggles all joints between being active or inactive. This method forces them to all have the 
     * same state
     */
    public void toggleAllActive() {
        // this method forces all joints into the same active state; subject to change
        allActive = !allActive;

        pivot.setActive(allActive);
        telescope.setActive(allActive);
        wrist.setActive(allActive);
    }

    /**
     * Changes the neutral mode of all arm motors to brake or coast
     */
    public void setBrakeMode(boolean brake) {
        pivot.setBrakeMode(brake);
        telescope.setBrakeMode(brake);
        wrist.setBrakeMode(brake);
    }

    public void setArmSide(ActiveArmSide side) {
        activeSide = side;
    }
    
    public ActiveArmSide getArmSide() {
        return activeSide;
    }

    public void invertActiveSide() {
        // java enums are lame
        // I hope the JVM optimizes this
        if (activeSide == ActiveArmSide.BACK) {
            setArmSide(ActiveArmSide.FRONT);
        } else {
            setArmSide(ActiveArmSide.BACK);
        }
    }

    /**
     * Builds a command that moves the arm to the setpoint in question
     * @param wait If this is true, the command will not finish until the arm is done moving. Otherwise,
     * the command finishes instantly.
     */
    public Command move(ArmPosition setpoint, boolean wait) {
        if (wait) {
            return new InstantCommand(() -> this.setSetpoint(setpoint))
                    .andThen(Commands.waitUntil(this::atSetpoint));
        }

        return new InstantCommand(() -> this.setSetpoint(setpoint));
    }

    public static record ArmPosition(double pivot, double telescope, double wrist) {
    }

    private static enum ArmMovementState {
        /**
         * Try to move everything to the setpoint position. Takes place as the last step of a large
         * movement, the only step of a small movement, and the default state in between movements.
         */
        DEFAULT,
        /**
         * Retract the telescope to avoid jolting the robot with a large movement.
         */
        RETRACT_TELESCOPE,
        /**
         * Move the pivot to the new setpoint position with the telescope retracted.
         */
        MOVE_PIVOT
    }

    public static enum ActiveArmSide {
        FRONT,
        BACK
    }
}
