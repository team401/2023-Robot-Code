package frc.robot.subsystems.arm.telescope;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.TelescopeConstants;
import frc.robot.subsystems.arm.GenericArmJoint;

public class Telescope extends GenericArmJoint {

    private final TelescopeIO io;
    private final TelescopeIOInputsAutoLogged inputs = new TelescopeIOInputsAutoLogged();

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
            TelescopeConstants.kS,
            TelescopeConstants.kV);
    private final PIDController feedbackController = new PIDController(TelescopeConstants.kP, 0, 0);

    private final DoubleSupplier pivotAngleSupplier;

    private final Timer homeTimer = new Timer();

    public Telescope(
            TelescopeIO io,
            TrapezoidProfile.Constraints constraints,
            DoubleSupplier pivotAngleSupplier,
            double range,
            double defaultSetpoint) {
        super(constraints, range, defaultSetpoint);

        this.io = io;
        this.pivotAngleSupplier = pivotAngleSupplier;
    }

    public Telescope(
            TelescopeIO io,
            TrapezoidProfile.Constraints constraints,
            DoubleSupplier pivotPositionSupplier,
            double range) {
        this(io, constraints, pivotPositionSupplier, range, TelescopeConstants.stowedPosition);
    }

    public Telescope(TelescopeIO io, TrapezoidProfile.Constraints constraints, DoubleSupplier pivotPositionSupplier) {
        this(io, constraints, pivotPositionSupplier, 0.02);
    }

    @Override
    public void updateInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("Telescope", inputs);
    }

    @Override
    public double getPosition() {
        return inputs.positionM;
    }

    @Override
    public double getVelocity() {
        return inputs.velocityMS;
    }

    @Override
    public void setBrakeMode(boolean brake) {
        io.setBrakeMode(brake);
    }

    @Override
    public void jogSetpointPositive() {
        setpoint = MathUtil.clamp(
                setpoint + 0.05,
                TelescopeConstants.minPosM,
                TelescopeConstants.maxPosM);
    }

    @Override
    public void jogSetpointNegative() {
        setpoint = MathUtil.clamp(
                setpoint - 0.05,
                TelescopeConstants.minPosM,
                TelescopeConstants.maxPosM);
    }

    @Override
    public void home() {
        homing = true;

        setOutput(-1.0);

        homeTimer.reset();
        homeTimer.start();
    }

    @Override
    protected boolean runHomingLogic() {
        if (inputs.statorCurrent < 30) {
            homeTimer.reset();
        }

        if (homeTimer.hasElapsed(0.3)) {
            io.setSensorPosition(0);

            homing = false;
            return true;
        }

        return false;
    }

    @Override
    protected double calculateControl(TrapezoidProfile.State setpoint) {
        return feedbackController.calculate(getPosition(), setpoint.position)
                + feedforward.calculate(setpoint.velocity)
                // Compensates for weight of telescope as the pivot goes up
                // Gravity Constant * Sine of Angle
                + TelescopeConstants.kG * Math.sin(pivotAngleSupplier.getAsDouble());
    }

    @Override
    protected void setOutput(double volts) {
        io.setOutput(volts);
    }

    @Override
    protected void resetControlLoop() {
        feedbackController.reset();
    }
}
