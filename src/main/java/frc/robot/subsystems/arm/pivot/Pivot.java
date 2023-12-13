package frc.robot.subsystems.arm.pivot;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.TelescopeConstants;
import frc.robot.subsystems.arm.GenericArmJoint;

public class Pivot extends GenericArmJoint {

    private final PivotIO io;
    private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

    public final PIDController feedbackController = new PIDController(
            PivotConstants.kP,
            0,
            PivotConstants.kD);
    private final ArmFeedforward feedforward = new ArmFeedforward(
            PivotConstants.kS,
            PivotConstants.kG,
            PivotConstants.kV,
            PivotConstants.kA);

    private final DoubleSupplier telescopePositionSupplier;

    public Pivot(
            PivotIO io,
            TrapezoidProfile.Constraints constraints,
            DoubleSupplier telescopePositionSupplier,
            double range,
            double defaultSetpoint) {
        super(constraints, range, defaultSetpoint);

        this.telescopePositionSupplier = telescopePositionSupplier;
        this.io = io;
    }

    public Pivot(
            PivotIO io,
            TrapezoidProfile.Constraints contraints,
            DoubleSupplier telescopePositionSupplier,
            double range) {
        this(io, contraints, telescopePositionSupplier, range, Math.PI / 2);
    }

    public Pivot(PivotIO io, TrapezoidProfile.Constraints contraints, DoubleSupplier telescopePositionSupplier) {
        this(io, contraints, telescopePositionSupplier, 0.03);
    }

    @Override
    public double getPosition() {
        return inputs.positionRad;
    }

    @Override
    public double getVelocity() {
        return inputs.velocityRadS;
    }

    @Override
    public void setBrakeMode(boolean brake) {
        io.setBrakeMode(brake);
    }

    @Override
    public void jogSetpointPositive() {
        setpoint = MathUtil.clamp(
                setpoint + Math.PI / 64,
                PivotConstants.maxFwdRotationRad,
                PivotConstants.maxBackRotationRad);
    }

    @Override
    public void jogSetpointNegative() {
        setpoint = MathUtil.clamp(
                setpoint - Math.PI / 64,
                PivotConstants.maxFwdRotationRad,
                PivotConstants.maxBackRotationRad);
    }

    @Override
    protected double calculateControl(TrapezoidProfile.State setpointState) {
        Logger.recordOutput("Pivot/control-setpoint", setpointState.position);
        return feedbackController.calculate(getPosition(), setpointState.position)
                + feedforward.calculate(setpointState.position, setpointState.velocity)
                // Compensates for telescope extention
                // Gravity constant * Telescope Extention (proportion) * Cosine of Angle
                + PivotConstants.extraKg
                        * telescopePositionSupplier.getAsDouble() / TelescopeConstants.maxPosM
                        * Math.cos(getPosition());
    }

    @Override
    protected void updateInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("Pivot", inputs);
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
