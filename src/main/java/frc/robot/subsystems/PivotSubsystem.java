package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.TelescopeConstants;

public class PivotSubsystem extends SubsystemBase {
    /**Primary Motor/ Leader */
    private TalonFX rightMotor = new TalonFX(CANDevices.rightPivotMotorID, CANDevices.canivoreName);
    private DutyCycleEncoder encoder = new DutyCycleEncoder(CANDevices.pivotEncoderID);
    private TalonFX leftMotor = new TalonFX(CANDevices.leftPivotMotorID, CANDevices.canivoreName);

    // For safety; detect when encoder stops sending new data
    private double lastEncoderPos;
    private int cycleCount = 0;
    private boolean dead = false;

    private double simPos;

    // The subsystem holds its own PID and feedforward controllers and provides calculations from
    // them, but cannot actually set its own motor output, as accurate feedforward calculations
    // require information from the telescope subsytem.
    public final PIDController controller = new PIDController(PivotConstants.kP, 0, 0);
    private final ArmFeedforward feedforward = new ArmFeedforward(
        PivotConstants.kS,
        PivotConstants.kG,
        PivotConstants.kV,
        PivotConstants.kA);
    private final TrapezoidProfile.Constraints constraintsRad = 
        new TrapezoidProfile.Constraints(
            Units.degreesToRadians(30),
            Units.degreesToRadians(10));

    // Stores the most recent setpoint to allow the Hold command to hold it in place
    private TrapezoidProfile.State currentSetpointRad = 
        new TrapezoidProfile.State(getPositionRad(), 0);


    public PivotSubsystem() {
        rightMotor.setInverted(InvertType.InvertMotorOutput);

        leftMotor.follow(rightMotor);
        leftMotor.setInverted(InvertType.OpposeMaster);

        leftMotor.setNeutralMode(NeutralMode.Coast);
        rightMotor.setNeutralMode(NeutralMode.Coast);

        rightMotor.configStatorCurrentLimit(
            new StatorCurrentLimitConfiguration(
                true,
            70,
            80,
            1)
        );
        rightMotor.configStatorCurrentLimit(
            new StatorCurrentLimitConfiguration(
                true,
            70,
            80,
            1)
        );

        SmartDashboard.putNumber("Pivot test setpoint", 0);
    }

    public double getPositionRad() {
        // return -(encoder.getAbsolutePosition() * 2 * Math.PI + PivotConstants.encoderOffsetRad);
        return encoder.getAbsolutePosition();
        // return simPos;
    }

    public double getVelRadS() {
        return rightMotor.getSelectedSensorVelocity() 
            * 4096 * 2 * Math.PI * 10 
            * PivotConstants.armToMotorGearRatio;
    }

    /**
     * @return Most recent setpoint set by a move command. This setpoint only exists for utility
     * purposes and is not used by the subsystem
     */
    public TrapezoidProfile.State getDesiredSetpointRad() {
        return currentSetpointRad;
    }

    public TrapezoidProfile.Constraints getConstraintsRad() {
        return constraintsRad;
    }

    /**
     * 
     * @param setpointRad
     * @param telescopePosM
     * @return
     */
    public double calculateControl(TrapezoidProfile.State setpointRad, double telescopePosM) {
        return controller.calculate(getPositionRad(), setpointRad.position)
            + feedforward.calculate(setpointRad.position, setpointRad.velocity)
            // Compensates for telescope extention
            // Gravity constant * Telescope Extention (proportion) * Cosine of Angle
            + PivotConstants.extraKg * telescopePosM / TelescopeConstants.maxPosM
                * Math.cos(getPositionRad());
    }

    /**
     * Resets the PID controller stored in this subsystem.
     */
    public void resetPID() {
        controller.reset();
    }

    /**
     * @param state The setpoint state the telescope should be driven to.
     * Has no effect on the function of this subsytem.
     */
    public void setDesiredSetpointRad(TrapezoidProfile.State stateRad) {
        currentSetpointRad = stateRad;
    }

    public void jogSetpointForward() {
        currentSetpointRad.position += Math.PI / 24;
    }

    public void jogSetpointBack() {
        currentSetpointRad.position -= Math.PI / 24;
    }

    public void setSimPos(double pos) {
        simPos = pos;
    }
 
    /**
     * Sets the output power of the motors in volts if the boundry conditions
     * are not exceeded. If the pivot is out of bounds or dead, the voltage
     * will be set to 0.
     * @param input The voltage to set
     */
    public void setVolts(double input) {
        if (!dead && withinSoftLimits(input)) {
            rightMotor.set(ControlMode.PercentOutput, input / 12);
            SmartDashboard.putNumber("Pivot Commanded V", input);
        } else {
            rightMotor.set(ControlMode.PercentOutput, 0);
        }
    }

    /**
     * Sets the output power of the motors in volts, even if it is past the max
     * rotations or dead. Voltage is divided in half. <p>
     * 
     * ONLY USED BY HUMANS. Do not have PID or feedforward use this method.
     * @param input The voltage to set
     */
    public void overrideVolts(double input) {
        rightMotor.set(ControlMode.PercentOutput, input / 24);
    }


    /**
     * 'Kills' the subsystem. The motor will be stopped and no loger respond to input
     */
    public void die() {
        setVolts(0);
        dead = true;
    }
    
    /**
     * 'Revives' the subsytem. If it is dead, the motor will start responding.<p>
     * DO NOT have regular code call this method. Only a human button should do this.
     */
    public void revive() {
        dead = false;
    }

    /**
     * @return a new InstantCommand that stops the motor and requires this subsystem
     */
    public Command killCommand() {
        return new InstantCommand(this::die, this);
    }



    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot Position", getPositionRad());
        SmartDashboard.putNumber("Right Pivot Current", rightMotor.getStatorCurrent());
        SmartDashboard.putNumber("Left Pivot Current", leftMotor.getStatorCurrent());
        SmartDashboard.putNumber("Pivot Desired Setpoint", currentSetpointRad.position);

        SmartDashboard.putBoolean("Pivot Dead", dead);

        SmartDashboard.putBoolean("At Back", RobotState.getInstance().atBack());

        RobotState.getInstance().putPivotDisplay(getPositionRad());

        if (DriverStation.isEnabled()) checkIfDead();
    }

    private void checkIfDead() {
        if (lastEncoderPos == encoder.getAbsolutePosition()) {
            cycleCount++;
            if (cycleCount > 25) {
                die();
                return;
            }
        } else {
            cycleCount = 0;
        }
        lastEncoderPos = encoder.getAbsolutePosition();
    }

    private boolean withinSoftLimits(double input) {
        if (input > 0 && getPositionRad() > PivotConstants.maxBackRotationRad) {
            return false;
        }
        if (input < 0 && getPositionRad() < PivotConstants.maxFwdRotationRad) {
            return false;
        }
        return true;
    }
}
