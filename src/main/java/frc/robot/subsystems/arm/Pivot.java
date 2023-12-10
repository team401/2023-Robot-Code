package frc.robot.subsystems.arm;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.TelescopeConstants;

public class Pivot extends GenericArmJoint {

    private TalonFX rightMotor = new TalonFX(CANDevices.rightPivotMotorID, CANDevices.canivoreName);
    private DutyCycleEncoder encoder = new DutyCycleEncoder(CANDevices.pivotEncoderID);
    private TalonFX leftMotor = new TalonFX(CANDevices.leftPivotMotorID, CANDevices.canivoreName);

    private double velocity = 0.0;
    private double lastPosition;

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
    TrapezoidProfile.Constraints constraints,
    DoubleSupplier telescopePositionSupplier,
    double range,
    double defaultSetpoint
    ) {
        super(constraints, range, defaultSetpoint);

        this.telescopePositionSupplier = telescopePositionSupplier;

        rightMotor.setInverted(false);

        leftMotor.setControl(new Follower(CANDevices.rightPivotMotorID, true));

        rightMotor.setControl(new StaticBrake());

        //I have no idea how to set a current limit from the API

        lastPosition = getPosition();
    }

    public Pivot(TrapezoidProfile.Constraints contraints, DoubleSupplier telescopePositionSupplier, double range) {
        this(contraints, telescopePositionSupplier, range, Math.PI / 2);
    }

    public Pivot(TrapezoidProfile.Constraints contraints, DoubleSupplier telescopePositionSupplier) {
        this(contraints, telescopePositionSupplier, 0.03);
    }

    @Override
    public double getPosition() {
        return encoder.getAbsolutePosition() * 2 * Math.PI + PivotConstants.encoderOffsetRad;
    }

    @Override
    public double getVelocity() {
        return velocity;
    }

    @Override
    public void setBrakeMode(boolean brake) {
        rightMotor.setControl(brake ? new StaticBrake() : new CoastOut());
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
        SmartDashboard.putNumber("Pivot/control-setpoint", setpointState.position);
        return feedbackController.calculate(getPosition(), setpointState.position)
                + feedforward.calculate(setpointState.position, setpointState.velocity)
                // Compensates for telescope extention
                // Gravity constant * Telescope Extention (proportion) * Cosine of Angle
                + PivotConstants.extraKg
                    * telescopePositionSupplier.getAsDouble() / TelescopeConstants.maxPosM
                        * Math.cos(getPosition());
    }

    @Override
    protected void update() {
        findVelocity();
    }

    @Override
    protected void setOutput(double volts) {
        rightMotor.setControl(new VoltageOut(volts));
    }

    @Override
    protected void resetControlLoop() {
        feedbackController.reset();
    }

    private void findVelocity() {
        velocity = (getPosition() - lastPosition) / Constants.loopTime;
        lastPosition = getPosition();
    }

}
