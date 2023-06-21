package frc.robot.subsystems.drive;

import java.security.InvalidParameterException;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveModulePosition;

public class ModuleIOFalcon500 implements ModuleIO {
    private final TalonFX driveMotor;
    private final TalonFX rotationMotor;
    private final CANCoder rotationEncoder;
    private final double initialOffsetRadians;

    //TODO: re-implement these checks
    //Check to see if encoder stops working
    private boolean killed = false;
    // private final Timer deadTimer;
    // private double lastRotationPosition = 0;

    public ModuleIOFalcon500(DriveModulePosition position) {
        switch(position) {
            case FRONT_LEFT:
                driveMotor = new TalonFX(CANDevices.frontLeftDriveMotorID, CANDevices.canivoreName);
                rotationMotor = new TalonFX(CANDevices.frontLeftRotationMotorID, CANDevices.canivoreName);
                rotationEncoder = new CANCoder(CANDevices.frontLeftRotationEncoderID, CANDevices.canivoreName);
                driveMotor.setInverted(false);
                initialOffsetRadians = DriveConstants.frontLeftAngleOffset;
            break;
            case FRONT_RIGHT:
                driveMotor = new TalonFX(CANDevices.frontRightDriveMotorID, CANDevices.canivoreName);
                rotationMotor = new TalonFX(CANDevices.frontRightRotationMotorID, CANDevices.canivoreName);
                rotationEncoder = new CANCoder(CANDevices.frontRightRotationEncoderID, CANDevices.canivoreName);
                driveMotor.setInverted(true);
                initialOffsetRadians = DriveConstants.frontRightAngleOffset;
            break;
            case BACK_LEFT:
                driveMotor = new TalonFX(CANDevices.backLeftDriveMotorID, CANDevices.canivoreName);
                rotationMotor = new TalonFX(CANDevices.backLeftRotationMotorID, CANDevices.canivoreName);
                rotationEncoder = new CANCoder(CANDevices.backLeftRotationEncoderID, CANDevices.canivoreName);
                driveMotor.setInverted(false);
                initialOffsetRadians = DriveConstants.backLeftAngleOffset;
            break;
            case BACK_RIGHT:
                driveMotor = new TalonFX(CANDevices.backRightDriveMotorID, CANDevices.canivoreName);
                rotationMotor = new TalonFX(CANDevices.backRightRotationMotorID, CANDevices.canivoreName);
                rotationEncoder = new CANCoder(CANDevices.backRightRotationEncoderID, CANDevices.canivoreName);
                driveMotor.setInverted(true);
                initialOffsetRadians = DriveConstants.backRightAngleOffset;
            break;
            default:
                // Java enums are dumb
                throw new InvalidParameterException("This should be unreachable. You screwed up.");
        }

        driveMotor.configFactoryDefault(1000);
        rotationMotor.configFactoryDefault(1000);
        setFramePeriods(driveMotor, true);
        setFramePeriods(rotationMotor, false);

        driveMotor.setNeutralMode(NeutralMode.Brake);
        rotationMotor.setNeutralMode(NeutralMode.Brake);
        
        rotationMotor.setInverted(true);
        
        driveMotor.configVoltageCompSaturation(12, 1000);
        driveMotor.enableVoltageCompensation(true);
        rotationMotor.configVoltageCompSaturation(12, 1000);
        rotationMotor.enableVoltageCompensation(true);

        driveMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 70, 80, 1));
        rotationMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 70, 80, 1));

        driveMotor.configNeutralDeadband(0, 1000);
        rotationMotor.configNeutralDeadband(0, 1000);

        rotationEncoder.configFactoryDefault(1000);
        rotationEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 20, 1000);
        rotationEncoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 255, 1000);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.drivePositionRad = driveMotor.getSelectedSensorPosition() / 2048.0 * 2.0 * Math.PI / DriveConstants.driveWheelGearReduction;
        inputs.driveVelocityRadPerSec = driveMotor.getSelectedSensorVelocity() / 2048 * 2.0 * Math.PI / DriveConstants.driveWheelGearReduction * 10;
        inputs.driveAppliedVolts = driveMotor.getMotorOutputVoltage();
        inputs.driveCurrentAmps = driveMotor.getStatorCurrent();

        inputs.rotationPositionRad = MathUtil.angleModulus(Units.degreesToRadians(rotationEncoder.getPosition())) - initialOffsetRadians;
        inputs.rotationVelocityRadPerSec = rotationEncoder.getVelocity();
        inputs.rotationAppliedVolts = rotationMotor.getMotorOutputVoltage();
        inputs.rotationCurrentAmps = rotationMotor.getStatorCurrent();
    }

    @Override
    public void zeroEncoders() {
        rotationEncoder.setPositionToAbsolute(1000);
        driveMotor.setSelectedSensorPosition(0, 0, 1000);
    }

    @Override
    public void setDriveVoltage(double volts) {
        if (!killed)
            driveMotor.set(ControlMode.PercentOutput, volts/12);
    }

    @Override
    public void setRotationVoltage(double volts) {
        if (!killed)
            rotationMotor.set(ControlMode.PercentOutput, volts/12);
    }

    @Override
    public void toggleKill() {
        killed = !killed;
        if (killed) {
            driveMotor.set(ControlMode.PercentOutput, 0);
            rotationMotor.set(ControlMode.PercentOutput, 0);
            driveMotor.setNeutralMode(NeutralMode.Coast);
            rotationMotor.setNeutralMode(NeutralMode.Coast);
        }
        else {
            driveMotor.setNeutralMode(NeutralMode.Brake);
            rotationMotor.setNeutralMode(NeutralMode.Brake);
        }
    }

    private static void setFramePeriods(TalonFX talon, boolean needMotorSensor) {
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255, 1000);
        if (!needMotorSensor) {
           talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255, 1000);
        }
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255, 1000);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255, 1000);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 255, 1000);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, 255, 1000);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 255, 1000);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 255, 1000);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 255, 1000);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 255, 1000);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255, 1000);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 255, 1000);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 255, 1000);
    }
}
