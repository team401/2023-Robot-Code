package frc.robot.subsystems.arm.pivot;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.PivotConstants;

public class PivotIOFalcon implements PivotIO {
    
    private TalonFX rightMotor = new TalonFX(CANDevices.rightPivotMotorID, CANDevices.canivoreName);
    private DutyCycleEncoder encoder = new DutyCycleEncoder(CANDevices.pivotEncoderID);
    private TalonFX leftMotor = new TalonFX(CANDevices.leftPivotMotorID, CANDevices.canivoreName);

    private double velocity = 0.0;
    private double lastPosition;

    public PivotIOFalcon() {
        rightMotor.setInverted(false);

        leftMotor.setControl(new Follower(CANDevices.rightPivotMotorID, true));

        rightMotor.setControl(new StaticBrake());

        //I have no idea how to set a current limit with the API

        lastPosition = getPosition();
    }

    @Override
    public void updateInputs(PivotIOInputsAutoLogged inputs) {
        updateVelocity();

        inputs.positionRad = getPosition();
        inputs.velocityRadS = velocity;
        inputs.appliedVolts = rightMotor.getMotorVoltage().getValueAsDouble();
        inputs.statorCurrent = rightMotor.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void setBrakeMode(boolean brake) {
        rightMotor.setControl(brake ? new StaticBrake() : new CoastOut());
    }

    @Override
    public void setOutput(double volts) {
        rightMotor.setControl(new VoltageOut(volts));
    }

    private double getPosition() {
        return encoder.getAbsolutePosition() * 2 * Math.PI + PivotConstants.encoderOffsetRad;
    }

    private void updateVelocity() {
        velocity = (getPosition() - lastPosition) / Constants.loopTime;
        lastPosition = getPosition();
    }
}
