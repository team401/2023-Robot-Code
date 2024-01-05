package frc.robot.subsystems.arm.wrist;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.CANDevices;
import frc.robot.Constants.WristConstants;

public class WristIOFalcon implements WristIO {

    private TalonFX motor = new TalonFX(CANDevices.wristMotorID);

    public WristIOFalcon() {
        TalonFXConfigurator config = motor.getConfigurator();
        config.apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
        config.apply(new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(70.0)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentThreshold(80.0)
            .withSupplyTimeThreshold(0.5));

        motor.setInverted(false);
    }

    @Override
    public void updateInputs(WristIOInputsAutoLogged inputs) {
        inputs.positionRad = motor.getPosition().getValueAsDouble() * 2 * Math.PI * WristConstants.gearRatio;
        inputs.velocityRadS = motor.getVelocity().getValueAsDouble() * 2 * Math.PI * WristConstants.gearRatio;
        inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
        inputs.statorCurrent = motor.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void setBrakeMode(boolean brake) {
        TalonFXConfigurator config = motor.getConfigurator();
        if (brake) {
            config.apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
        } else {
            config.apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));
        }
    }

    @Override
    public void setOutput(double volts) {
        motor.setControl(new VoltageOut(volts));
    }

    @Override
    public void setSensorPosition(double position) {
        motor.setPosition(position / (2 * Math.PI) / WristConstants.gearRatio);

    }
    
}
