package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.CANDevices;


//TODO: Untested
public class IntakeIOSparkMax implements IntakeIO {
    private CANSparkMax leftMotor = new CANSparkMax(
        CANDevices.leftIntakeMotorID, MotorType.kBrushed);
    private CANSparkMax rightMotor = new CANSparkMax(
        CANDevices.rightIntakeMotorID, MotorType.kBrushed);

    public IntakeIOSparkMax() {
        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();
        
        leftMotor.setSmartCurrentLimit(40);
        rightMotor.setSmartCurrentLimit(40);

        leftMotor.setInverted(true);
        rightMotor.setInverted(false);
        
        leftMotor.burnFlash();
        rightMotor.burnFlash();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.leftOutputVolts = leftMotor.getAppliedOutput();
        inputs.rightOutputVolts = rightMotor.getAppliedOutput();

        inputs.leftCurrentAmps = leftMotor.getOutputCurrent();
        inputs.rightCurrentAmps = rightMotor.getOutputCurrent();
    }

    @Override
    public void setWheelVelocitiesVolts(double left, double right) {
        leftMotor.set(left / 12);
        rightMotor.set(right / 12);
    }
}
