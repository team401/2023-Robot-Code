package frc.robot.subsystems.pivot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.PivotConstants;

// TODO: Untested
public class PivotIOFalcon implements PivotIO {
    /**Primary Motor/ Leader */
    private TalonFX rightMotor = new TalonFX(CANDevices.rightPivotMotorID, CANDevices.canivoreName);
    private DutyCycleEncoder encoder = new DutyCycleEncoder(CANDevices.pivotEncoderID);
    private TalonFX leftMotor = new TalonFX(CANDevices.leftPivotMotorID, CANDevices.canivoreName);

    private double lastEncoderPos = 0.0;

    public PivotIOFalcon() {
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
            1)
        );
        rightMotor.configStatorCurrentLimit(
            new StatorCurrentLimitConfiguration(
                true,
            70,
            80,
            1)
        );
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        inputs.positionRad = encoder.getAbsolutePosition() * 2 * Math.PI + PivotConstants.encoderOffsetRad;
        inputs.velocityRadPerSec = (inputs.positionRad - lastEncoderPos) / 0.02;
        lastEncoderPos = inputs.positionRad;

        inputs.appliedVoltsLeft = leftMotor.getMotorOutputVoltage();
        inputs.currentAmpsLeft = leftMotor.getStatorCurrent();
        inputs.appliedVoltsRight = rightMotor.getMotorOutputVoltage();
        inputs.currentAmpsRight = rightMotor.getStatorCurrent();
    }

    @Override
    public void setVolts(double volts) {
        rightMotor.set(ControlMode.PercentOutput, volts / 12.0);
    }

    @Override
    public void setBrakeMode(boolean braked) {
        leftMotor.setNeutralMode(braked ? NeutralMode.Brake : NeutralMode.Coast);
        rightMotor.setNeutralMode(braked ? NeutralMode.Brake : NeutralMode.Coast);
    }
}
