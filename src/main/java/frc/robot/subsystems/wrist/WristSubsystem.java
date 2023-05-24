package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.Constants.WristConstants;

public class WristSubsystem extends SubsystemBase {
    private final WristIO io;
    private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

    public boolean homed = false;

    public boolean atGoal = false;

    private boolean dead = false;

    // The subsystem holds its own PID and feedforward controllers and provides calculations from
    // them, but cannot actually set its own io output, as accurate feedforward calculations
    // require information from the pivot subsytem.
    private final TrapezoidProfile.Constraints constraintsRad = new TrapezoidProfile.Constraints(
        Units.degreesToRadians(630),
        Units.degreesToRadians(810));

    // TODO: Put this back

    // private final PIDController controller = new PIDController(WristConstants.kP, WristConstants.kI, 0);
    private final PIDController controller = new PIDController(1, 0, 0);
    // private final PIDController controllerHold = new PIDController(WristConstants.kPHold, WristConstants.kIHold, 0);
    private final PIDController controllerHold = new PIDController(1, 0, 0);

    private final ArmFeedforward feedforward = new ArmFeedforward(
        WristConstants.kS,
        WristConstants.kG,
        WristConstants.kV,
        WristConstants.kA);

    // Stores the most recent setpoint to allow the Hold command to hold it in place
    private TrapezoidProfile.State currentSetpointRad = new TrapezoidProfile.State();

    public WristSubsystem(WristIO io) {
        this.io = io;

        controller.setTolerance(0.05);
        controllerHold.setTolerance(0.05);
    }

    public double getPositionRad() {
        return inputs.positionRad;
    }

    public double getVelRadS() {
        return inputs.velocityRadPerSec;
    }

    public double getAmps() {
        return Math.abs(inputs.currentAmps);
    }

    public void setBrakeMode(boolean braked) {
        io.setBrakeMode(braked);
    }

    /**
     * @return Most recent setpoint set by a move command. This setpoint only exists for utility
     * purposes and is not used by the subsystem
     */
    public TrapezoidProfile.State getDesiredSetpointRad() {
        return currentSetpointRad;
    }

    public void jogSetpointForward() {
        currentSetpointRad.position += Math.PI / 24;
    }

    public void jogSetpointBack() {
        currentSetpointRad.position -= Math.PI / 24;
    }
    
    public TrapezoidProfile.Constraints getConstraintsRad() {
        return constraintsRad;
    }

    public void toggleKill() {
        io.setVolts(0);
        dead = !dead;
    }

    public void setCurrentLimit(double currentLimit, double triggerThresholdCurrent, double triggerThresholdTime) {
        io.setCurrentLimit(currentLimit, triggerThresholdCurrent, triggerThresholdTime);

    }

    /**
     * Does control calculations from its ArmFeedforward and PID controllers.
     * Does not command any ios.
     * @param setpoint The given setpoint, field-relative
     * @param angleDeg The angle of the wrist, also field-relative. Do not give
     * direct sensor values
     * @return The result of the calculation
     */
    public double calculateControl(TrapezoidProfile.State setpointRad, double angleRad, boolean holding) {

        double fb = holding ? controllerHold.calculate(angleRad, setpointRad.position) : controller.calculate(angleRad, setpointRad.position);
        double ff = feedforward.calculate(setpointRad.position, setpointRad.velocity);

        return fb + ff;
        
    }

    public double calculateControl(TrapezoidProfile.State setpointRad, double angleRad, boolean holding, double pivotVel) {

        double fb = holding ? controllerHold.calculate(angleRad, setpointRad.position) : controller.calculate(angleRad, setpointRad.position);
        double ff = feedforward.calculate(setpointRad.position, setpointRad.velocity + pivotVel);

        return fb + ff;
        
    }

    /**
     * Resets the PID controller stored in this subsystem.
     */
    public void resetPID() {
        controller.reset();
        controllerHold.reset();
    }

    /**
     * @param state The setpoint state the wrist should be driven to.
     * Has no effect on the function of this subsytem.
     */
    public void updateDesiredSetpointRad(TrapezoidProfile.State state) {
        currentSetpointRad = state;
    }

    /**
     * For homing. Resets the encoder offset to the position of 141 degrees,
     * the position the wrist should be at when it goes all the way to the arm.
     */
    public void resetOffset() {
        inputs.positionRad = 2.81 / (2 * Math.PI) * 2048 / WristConstants.gearRatio;
    }

    public void resetOffsetCube() {
        inputs.positionRad = 1.5 / (2 * Math.PI) * 2048 / WristConstants.gearRatio;
    }

    public void zeroOffset() {
        inputs.positionRad = 0;
    }

    public void setVolts(double input) {
        if (!dead) {
            io.setVolts(input);
            return;
        }
        io.setVolts(0);
    }

    public void overrideVolts(double input) {
        System.out.println(input / 12);
        io.setVolts(input);
    }

    public void stop() {
        io.setVolts(0);
    }

    public void periodic() {

        // SmartDashboard.putNumber("Wrist Position", getPositionRad());
        // SmartDashboard.putNumber("Wrist Desired Position", currentSetpointRad.position);
        // SmartDashboard.putNumber("Wrist Amps", getAmps());
        // SmartDashboard.putBoolean("Wrist Dead", dead);
        // SmartDashboard.putNumber("Wrist Input Voltage", io.getio)
        // SmartDashboard.putNumber("Wrist Voltage", io.getioOutputVoltage());
        // SmartDashboard.putBoolean("Wrist Homed", homed);
        // SmartDashboard.putNumber("Wrist Velocity", getVelRadS());

        io.updateInputs(inputs);

        Logger.getInstance().processInputs("Wrist", inputs);

        RobotState.getInstance().putWristDisplay(getPositionRad());
    }
}