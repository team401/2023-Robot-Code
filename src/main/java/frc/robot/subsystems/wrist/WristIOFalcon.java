package frc.robot.subsystems.wrist;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.WristConstants;

// TODO: Untested
public class WristIOFalcon implements WristIO {
    private TalonFX motor = new TalonFX(CANDevices.wristMotorID);

    public WristIOFalcon() {
        motor.setInverted(false);
        motor.setInverted(InvertType.None);

        motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        motor.setNeutralMode(NeutralMode.Brake);

        motor.configNeutralDeadband(0.004);
        
        motor.configStatorCurrentLimit(
            new StatorCurrentLimitConfiguration(true, 70, 80, 0.5));
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.appliedVolts = motor.getMotorOutputVoltage();
        inputs.positionRad = motor.getSelectedSensorPosition() / 2048 * 2 * Math.PI * WristConstants.gearRatio;
        inputs.velocityRadPerSec = motor.getSelectedSensorVelocity() / 2048 * 2 * Math.PI * 10 * WristConstants.gearRatio;
        inputs.currentAmps = motor.getStatorCurrent();
    }

    @Override
    public void setVolts(double volts) {
        motor.set(ControlMode.PercentOutput, volts / 12.0);
    }

    @Override
    public void setBrakeMode(boolean braked) {
        motor.setNeutralMode(braked ? NeutralMode.Brake : NeutralMode.Coast);
    }

    @Override
    public void setCurrentLimit(double currentLimit, double triggerThresholdCurrent, double triggerThresholdTime) {
        motor.configStatorCurrentLimit(
            new StatorCurrentLimitConfiguration(true, currentLimit, triggerThresholdCurrent, triggerThresholdTime));

    }

    @Override
    public void configurePID(double kP, double kI, double kD) {
        motor.config_kP(0, kP);
        motor.config_kI(0, kI);
        motor.config_kD(0, kD);
        motor.config_kF(0, 0.0);
    }
}
