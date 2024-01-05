package frc.robot.subsystems.arm.telescope;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.TelescopeConstants;

public class TelescopeIOFalcon implements TelescopeIO {

    private TalonFX motor = new TalonFX(CANDevices.telescopeMotorID);

    public TelescopeIOFalcon() {
        TalonFXConfigurator config = motor.getConfigurator();
        config.apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
        config.apply(new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(70.0)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentThreshold(80.0)
            .withSupplyTimeThreshold(0.5));

        motor.setInverted(false);

        motor.setPosition(0);
    }

    @Override
    public void updateInputs(TelescopeIOInputsAutoLogged inputs) {
        inputs.positionM = motor.getPosition().getValueAsDouble() * 2 * Math.PI
            * TelescopeConstants.conversionM;
        inputs.velocityMS = motor.getVelocity().getValueAsDouble() * 2 * Math.PI
            * TelescopeConstants.conversionM;
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
        motor.setControl(new VoltageOut(MathUtil.clamp(volts, -3, 3)));
    }
    
    @Override
    public void setSensorPosition(double pos) {
        motor.setPosition(pos / ((2 * Math.PI) * TelescopeConstants.conversionM));
    }
}
