package frc.robot.subsystems.telescope;

import org.littletonrobotics.junction.Logger;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ArmManager;
import frc.robot.Constants.TelescopeConstants;

public class TelescopeSubsystem extends SubsystemBase{
    private final TelescopeIO io;
    private final TelescopeIOInputsAutoLogged inputs = new TelescopeIOInputsAutoLogged();

    public boolean homed = false;

    private boolean dead = false;

    public boolean atGoal = false;

    // The subsystem holds its own PID and feedforward controllers and provides calculations from
    // them, but cannot actually set its own motor output, as accurate feedforward calculations
    // require information from the pivot subsytem.
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
        TelescopeConstants.kS,
        TelescopeConstants.kV);
    private final TrapezoidProfile.Constraints constraints = 
        new TrapezoidProfile.Constraints(3, 3);
    private final PIDController controller = 
        new PIDController(TelescopeConstants.kP, 0, 0);
        
    // Stores the most recent setpoint to allow the Hold command to hold it in place
    private TrapezoidProfile.State currentSetpoint = new TrapezoidProfile.State(0.06, 0);

    public TelescopeSubsystem(TelescopeIO io) {
        this.io = io;

        if(Constants.mode == Constants.Mode.SIM)
            homed = true;

        // SmartDashboard.putNumber("Telescope test setpoint", 0);
    }

    public double getPositionM() {
        // 4096 units per rotation, multiply rotations by diameter
        return inputs.positionMeters;
    }

    public double getVel() {
        return inputs.velocityMetersPerSec;
    }

    public double getAmps() {
        return inputs.currentAmps;
    }

     /**
     * @return Most recent setpoint set by a move command. This setpoint only exists for utility
     * purposes and is not used by the subsystem
     */
    public TrapezoidProfile.State getDesiredSetpoint() {
        return currentSetpoint;
    }

    public TrapezoidProfile.Constraints getConstraints() {
        return constraints;
    }

    public void toggleKill() {
        io.setVolts(0);
        dead = !dead;
    }

    /**
     * Does control calculations from its ArmFeedforward and PID controllers.
     * Does not command any motors.
     * @param setpoint The given setpoint
     * @param pivotAngleRad The position of the pivot, required for accurate feedforward
     * @return The result of the calculation. Add kG * sine of the pivot angle
     */
    public double calculateControl(TrapezoidProfile.State setpoint, double pivotAngleRad) {
        // SmartDashboard.putNumber("Telescope calculated", controller.calculate(getPositionM(), setpoint.position));
        return controller.calculate(getPositionM(), setpoint.position)
            + feedforward.calculate(setpoint.velocity)
            // Compensates for weight of telescope as the pivot goes up
            // Gravity Constant * Sine of Angle
            + TelescopeConstants.kG * Math.sin(pivotAngleRad);
    }

    /**
     * Resets the PID controller stored in this subsystem.
     */
    public void resetPID() {
        controller.reset();
    }

    /**
     * @param state The setpoint state the telescope should be driven to.
     * Has no effect on the function of this subsytem.
     */
    public void setDesiredSetpoint(TrapezoidProfile.State state) {
        currentSetpoint = state;
    }

    public void jogSetpointForward() {
        currentSetpoint.position = 
            MathUtil.clamp(
                    currentSetpoint.position + 0.05,
                    TelescopeConstants.minPosMeters,
                    TelescopeConstants.maxPosMeters);
    }

    public void jogSetpointBackward() {
        currentSetpoint.position = 
            MathUtil.clamp(
                    currentSetpoint.position - 0.05,
                    TelescopeConstants.minPosMeters,
                    TelescopeConstants.maxPosMeters);
    }

    public void setVolts(double input) {
        if (!dead)
        io.setVolts(input);
    }

    public void overrideVolts(double input) {
        io.setVolts(input);
    }

    public void stop() {
        io.setVolts(0);
    }

    public void setBrakeMode(boolean braked) {
        io.setBrakeMode(braked);
    }

    public void resetOffset() {
        io.setOffset(0);
    }

    public void setP(double p) {
        controller.setP(p);
        controller.reset();
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Telescope Position", getPositionM()); 
        // SmartDashboard.putNumber("Telescope Velocity", getVel());
        // SmartDashboard.putNumber("Telescope Desired Position", currentSetpoint.position);
        // SmartDashboard.putBoolean("Telescope Dead", dead);
        // SmartDashboard.putNumber("Telescope Voltage", motor.getMotorOutputVoltage());
        // SmartDashboard.putNumber("Telescope Amps", getAmps());

        io.updateInputs(inputs);

        Logger.getInstance().processInputs("Telescope", inputs);

        ArmManager.getInstance().putTelescopeDisplay(getPositionM());
    }
}
