package frc.robot.subsystems.arm.telescope;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.CANDevices;
import frc.robot.Constants.TelescopeConstants;

public class TelescopeIOFalcon implements TelescopeIO {

    private TalonFX motor = new TalonFX(CANDevices.telescopeMotorID);

    public TelescopeIOFalcon() {
        motor.setInverted(false);
        motor.setControl(new StaticBrake());
        
        //I have no idea how to set a current limit from the API
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
        motor.setControl(brake ? new StaticBrake() : new CoastOut());
    }

    @Override
    public void setOutput(double volts) {
        motor.setControl(new VoltageOut(volts));
    }
    
    @Override
    public void setSensorPosition(double pos) {
        motor.setPosition(pos / (2 * Math.PI) / TelescopeConstants.conversionM);
    }
}
