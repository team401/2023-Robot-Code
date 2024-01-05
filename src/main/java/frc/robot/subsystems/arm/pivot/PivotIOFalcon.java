package frc.robot.subsystems.arm.pivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.PivotConstants;

public class PivotIOFalcon implements PivotIO {
    
    private TalonFX rightMotor = new TalonFX(CANDevices.rightPivotMotorID, CANDevices.canivoreName);
    private DutyCycleEncoder encoder = new DutyCycleEncoder(CANDevices.pivotEncoderID);
    private TalonFX leftMotor = new TalonFX(CANDevices.leftPivotMotorID, CANDevices.canivoreName);

    private StatusSignal<Double> appliedVoltageSignal;
    private StatusSignal<Double> statorCurrentSignal;

    private double velocity = 0.0;
    private double lastPosition;

    public PivotIOFalcon() {
        configureMotors();

        rightMotor.setInverted(false);

        leftMotor.setControl(new Follower(CANDevices.rightPivotMotorID, true));

        lastPosition = getPosition();
    }

    private void configureMotors() {
        TalonFXConfigurator rightConfig = rightMotor.getConfigurator();

        // set to brake mode
        rightConfig.apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
        // set a current limit of 70A and a burst current limit of 80A
        rightConfig.apply(new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(70.0)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentThreshold(80.0)
            .withSupplyTimeThreshold(1.0));

        TalonFXConfigurator leftConfig = leftMotor.getConfigurator();

        // set to brake mode
        leftConfig.apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
        // set a current limit of 70A and a burst current limit of 80A
        leftConfig.apply(new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(70.0)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentThreshold(80.0)
            .withSupplyTimeThreshold(1.0));

        appliedVoltageSignal = rightMotor.getMotorVoltage();
        statorCurrentSignal = rightMotor.getStatorCurrent();
    }

    @Override
    public void updateInputs(PivotIOInputsAutoLogged inputs) {
        updateVelocity();
        BaseStatusSignal.refreshAll(appliedVoltageSignal, statorCurrentSignal);

        inputs.positionRad = getPosition();
        inputs.velocityRadS = velocity;
        inputs.appliedVolts = appliedVoltageSignal.getValueAsDouble();
        inputs.statorCurrent = statorCurrentSignal.getValueAsDouble();
    }

    @Override
    public void setBrakeMode(boolean brake) {
        //TODO: find a more effient way to do this
        if (brake) {
            TalonFXConfigurator rightConfig = rightMotor.getConfigurator();
            rightConfig.apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));

            TalonFXConfigurator leftConfig = leftMotor.getConfigurator();
            leftConfig.apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
        } else {
            TalonFXConfigurator rightConfig = rightMotor.getConfigurator();
            rightConfig.apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));

            TalonFXConfigurator leftConfig = leftMotor.getConfigurator();
            leftConfig.apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));
        }
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
