package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.DriveConstants;

public class DriveModule {

    public double driveVelocityRadPerS;
    public double drivePositionRad;
    public double rotationPositionRad;

    private final TalonFX driveMotor;
    private final TalonFX rotationMotor;
    private final CANCoder rotationEncoder;
    private final double initialOffsetRadians;

    private static void setFramePeriods(TalonFX talon, boolean needMotorSensor) {
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255, 1000);
        //if (!needMotorSensor) {
        //    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255, 1000);
        //}
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

    public DriveModule(int driveMotorID, int rotationMotorID, int cancoderID, double measuredOffsetsRadians) {

        driveMotor = new TalonFX(driveMotorID, CANDevices.canivoreName);
        rotationMotor = new TalonFX(rotationMotorID, CANDevices.canivoreName);
        rotationEncoder = new CANCoder(cancoderID, CANDevices.canivoreName);
        

        driveMotor.configFactoryDefault(1000);
        rotationMotor.configFactoryDefault(1000);
        setFramePeriods(driveMotor, true);
        setFramePeriods(rotationMotor, false);

        driveMotor.setNeutralMode(NeutralMode.Brake);
        rotationMotor.setNeutralMode(NeutralMode.Brake);

        driveMotor.setInverted(true);
        
        driveMotor.configVoltageCompSaturation(12, 1000);
        driveMotor.enableVoltageCompensation(true);
        rotationMotor.configVoltageCompSaturation(12, 1000);
        rotationMotor.enableVoltageCompensation(true);

        driveMotor.configNeutralDeadband(0, 1000);
        rotationMotor.configNeutralDeadband(0, 1000);

        rotationEncoder.configFactoryDefault(1000);
        rotationEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 20, 1000);
        rotationEncoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 255, 1000);

        initialOffsetRadians = measuredOffsetsRadians;

    }

    public void updateVariables() {
        
        drivePositionRad = driveMotor.getSelectedSensorPosition() 
            / 2048.0 * 2.0 * Math.PI / DriveConstants.driveWheelGearReduction;

        driveVelocityRadPerS = driveMotor.getSelectedSensorVelocity() 
            / 2048.0 * 10.0 * 2.0 * Math.PI / DriveConstants.driveWheelGearReduction;
            
        //Using relative encoder in the CANCoder
        rotationPositionRad = Units.degreesToRadians(rotationEncoder.getPosition()) - initialOffsetRadians;

    }

    public void zeroEncoders() {
        rotationEncoder.setPositionToAbsolute(1000);
        driveMotor.setSelectedSensorPosition(0, 0, 1000);
    }

    public void setRotationVoltage(double volts) {
        rotationMotor.set(ControlMode.PercentOutput, volts/12);
    }

    public void setDriveVoltage(double volts) {
        driveMotor.set(ControlMode.PercentOutput, volts/12);
    }

    public void setDriveVelocity(double velocityRadPerS, double ffVolts) {
        // Convert rad/s to motor velocity
        double velocityTicksPer100ms = velocityRadPerS * 2048.0 / 10.0 / 2.0 / Math.PI * DriveConstants.driveWheelGearReduction;

        driveMotor.set(ControlMode.Velocity, velocityTicksPer100ms, DemandType.ArbitraryFeedForward, ffVolts / 12.0);
    }

    public void setDrivePD(double p, double d) {
        driveMotor.config_kP(0, p);
        driveMotor.config_kD(0, d);
    }
    
}
