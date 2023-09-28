package frc.robot.subsystems.telescope;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.Constants.CANDevices;
import frc.robot.Constants.TelescopeConstants;

public class TelescopeIOFalcon implements TelescopeIO {

    private TalonFX motor = new TalonFX(CANDevices.telescopeMotorID);
    
    public TelescopeIOFalcon() {
        motor.setInverted(InvertType.None);
        motor.setNeutralMode(NeutralMode.Brake);
        
        motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        motor.setSensorPhase(false);

        motor.configNeutralDeadband(0.004);

        motor.configStatorCurrentLimit(
            new StatorCurrentLimitConfiguration(true, 40, 50, 0.5));
    }

    @Override
    public void updateInputs(TelescopeIOInputs inputs) {
        inputs.appliedVolts = motor.getMotorOutputVoltage();
        inputs.positionMeters = motor.getSelectedSensorPosition() / 4096 * 2 * Math.PI  * TelescopeConstants.conversionM;
        inputs.velocityMetersPerSec = motor.getSelectedSensorVelocity() / 4096 * 2 * Math.PI * TelescopeConstants.conversionM * 10;
        inputs.currentAmps = Math.abs(motor.getStatorCurrent());
    }

    @Override
    public void setBrakeMode(boolean braked) {
        motor.setNeutralMode(braked ? NeutralMode.Brake : NeutralMode.Coast);
    }

    @Override
    public void setOffset(double offset) {
        motor.setSelectedSensorPosition(offset);
    }

    @Override
    public void setVolts(double volts) {
        motor.set(ControlMode.Current, volts);
    }

    @Override
    public void setCurrentLimit(double currentLimit, double triggerThresholdCurrent, double triggerThresholdTime) {
        motor.configStatorCurrentLimit(
            new StatorCurrentLimitConfiguration(true, currentLimit, triggerThresholdCurrent, triggerThresholdTime));
    }
}
