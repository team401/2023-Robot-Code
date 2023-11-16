package frc.robot.subsystems.arm;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.TelescopeConstants;

public class Pivot extends GenericArmJoint {

    private TalonFX rightMotor = new TalonFX(CANDevices.rightPivotMotorID, CANDevices.canivoreName);
    private DutyCycleEncoder encoder = new DutyCycleEncoder(CANDevices.pivotEncoderID);
    private TalonFX leftMotor = new TalonFX(CANDevices.leftPivotMotorID, CANDevices.canivoreName);

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
    double defaultSetpoint
    ) {
        super(constraints, defaultSetpoint);

        this.telescopePositionSupplier = telescopePositionSupplier;

        rightMotor.setInverted(InvertType.None);

        leftMotor.follow(rightMotor);
        leftMotor.setInverted(InvertType.OpposeMaster);

        leftMotor.configNeutralDeadband(0.004);
        rightMotor.configNeutralDeadband(0.004);

        leftMotor.setNeutralMode(NeutralMode.Brake);
        rightMotor.setNeutralMode(NeutralMode.Brake);

        rightMotor.configStatorCurrentLimit(
                new StatorCurrentLimitConfiguration(
                        true,
                        70,
                        80,
                        1));
        rightMotor.configStatorCurrentLimit(
                new StatorCurrentLimitConfiguration(
                        true,
                        70,
                        80,
                        1));
    }

    public Pivot(TrapezoidProfile.Constraints contraints, DoubleSupplier telescopePositionSupplier) {
        this(contraints, telescopePositionSupplier, Math.PI / 2);
    }

    @Override
    public double getPosition() {
        return encoder.getAbsolutePosition() * 2 * Math.PI + PivotConstants.encoderOffsetRad;
    }

    @Override
    public double getVelocity() {
        return 0.0; // TODO: implement velocity calculation
    }

    @Override
    protected double calculateControl(TrapezoidProfile.State setpointState) {
        return feedbackController.calculate(getPosition(), setpointState.position)
                + feedforward.calculate(setpointState.position, setpointState.velocity)
                // Compensates for telescope extention
                // Gravity constant * Telescope Extention (proportion) * Cosine of Angle
                + PivotConstants.extraKg
                    * telescopePositionSupplier.getAsDouble() / TelescopeConstants.maxPosM
                        * Math.cos(getPosition());
    }

    @Override
    protected void setOutput(double volts) {
        rightMotor.set(ControlMode.PercentOutput, volts / 12);
    }

    @Override
    protected void resetControlLoop() {
        feedbackController.reset();
    }

}
