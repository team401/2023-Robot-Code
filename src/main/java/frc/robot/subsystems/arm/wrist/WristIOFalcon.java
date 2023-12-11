package frc.robot.subsystems.arm.wrist;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.CANDevices;
import frc.robot.Constants.WristConstants;

public class WristIOFalcon implements WristIO {

    private TalonFX motor = new TalonFX(CANDevices.wristMotorID);

    public WristIOFalcon() {
        motor.setInverted(false);

        motor.setControl(new StaticBrake());

        //I have no idea how to set a current limit from the API
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
        motor.setControl(brake ? new StaticBrake() : new CoastOut());
    }

    @Override
    public void setOuput(double volts) {
        motor.setControl(new VoltageOut(volts));
    }

    @Override
    public void setSensorPosition(double position) {
        motor.setPosition(position / (2 * Math.PI) / WristConstants.gearRatio);

    }
    
}
