package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.WristConstants;

public class WristSubsystem extends SubsystemBase {
    private TalonFX motor = new TalonFX(CANDevices.wristMotorID);

    public boolean homed = false;

    // For safety; detect when encoder stops sending new data
    private double lastEncoderPos;
    private boolean dead = false;

    // The subsystem holds its own PID and feedforward controllers and provides calculations from
    // them, but cannot actually set its own motor output, as accurate feedforward calculations
    // require information from the pivot subsytem.
    private final PIDController controller = new PIDController(WristConstants.kP, WristConstants.kI, 0);
    private final ArmFeedforward feedforward = new ArmFeedforward(
        WristConstants.kS,
        WristConstants.kG,
        WristConstants.kV);
    private final TrapezoidProfile.Constraints constraintsRad = new TrapezoidProfile.Constraints(
        Units.degreesToRadians(360),
        Units.degreesToRadians(360));

    // Stores the most recent setpoint to allow the Hold command to hold it in place
    private TrapezoidProfile.State currentSetpointRad = new TrapezoidProfile.State();

    private double simPos = 0.0;

    public WristSubsystem() {
        motor.setInverted(false);
        motor.setInverted(InvertType.None);

        motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        motor.setNeutralMode(NeutralMode.Brake);
        

        controller.setTolerance(0.05);
    }

    public double getPositionRad() {
        return motor.getSelectedSensorPosition() / 2048 * 2 * Math.PI / 26 * Math.PI;
            // * WristConstants.gearRatio;
        // return simPos;
    }

    public double getVelRadS() {
        return motor.getSelectedSensorVelocity() / 2048 * 2 * Math.PI * 10;
    }

    public double getAmps() {
        return Math.abs(motor.getStatorCurrent());
    }

    /**
     * @return Most recent setpoint set by a move command. This setpoint only exists for utility
     * purposes and is not used by the subsystem
     */
    public TrapezoidProfile.State getDesiredSetpointRad() {
        return currentSetpointRad;
    }

    public void jogSetpointForward() {
        currentSetpointRad.position += Math.PI / 24;
    }

    public void jogSetpointBack() {
        currentSetpointRad.position -= Math.PI / 24;
    }
    
    public TrapezoidProfile.Constraints getConstraintsRad() {
        return constraintsRad;
    }

    /**
     * Does control calculations from its ArmFeedforward and PID controllers.
     * Does not command any motors.
     * @param setpoint The given setpoint, field-relative
     * @param angleDeg The angle of the wrist, also field-relative. Do not give
     * direct sensor values
     * @return The result of the calculation
     */
    public double calculateControl(TrapezoidProfile.State setpointRad, double angleRad) {
        return controller.calculate(angleRad, setpointRad.position)
        + feedforward.calculate(setpointRad.position, setpointRad.velocity);
    }

    /**
     * Resets the PID controller stored in this subsystem.
     */
    public void resetPID() {
        controller.reset();
    }

    /**
     * @param state The setpoint state the wrist should be driven to.
     * Has no effect on the function of this subsytem.
     */
    public void updateDesiredSetpointRad(TrapezoidProfile.State state) {
        currentSetpointRad = state;
    }

    public void setSimPosRad(double pos) {
        simPos = pos;
    }

    /**
     * For homing. Resets the encoder offset to the position of 141 degrees,
     * the position the wrist should be at when it goes all the way to the arm.
     */
    public void resetOffset() {
        motor.setSelectedSensorPosition(2.97 / (2 * Math.PI) * 2048 * 26 / Math.PI);
    }

    public void setVolts(double input) {
        if (!dead) {
            if (getPositionRad() > WristConstants.positiveLimitRad && input > 0 && homed) return;
            if (getPositionRad() < WristConstants.negativeLimitRad && input < 0 && homed) return;

            motor.set(ControlMode.PercentOutput, input / 12);
            return;
        }
        motor.set(ControlMode.PercentOutput, 0);
    }

    public void overrideVolts(double input) {
        motor.set(ControlMode.PercentOutput, input / 12);
    }

    public void stop() {
        motor.set(ControlMode.PercentOutput, 0);
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

    public void periodic() {

        SmartDashboard.putNumber("Wrist Position", getPositionRad());
        SmartDashboard.putNumber("Wrist Desired Position", currentSetpointRad.position);
        SmartDashboard.putNumber("Wrist Amps", getAmps());
        SmartDashboard.putBoolean("Wrist Dead", dead);
        SmartDashboard.putNumber("Wrist Voltage", motor.getMotorOutputVoltage());
        SmartDashboard.putBoolean("Wrist Homed", homed);

        RobotState.getInstance().putWristDisplay(getPositionRad());

        // checkIfDead();
    }

    private void checkIfDead() {
        // if (lastEncoderPos == wrist.getSelectedSensorPosition()) {
        //     cycleCounter++;
        //     if (cycleCounter > 25) {
        //         die();
        //         return;
        //     }
        // } else {
        //     cycleCounter = 0;
        // }
        lastEncoderPos = motor.getSelectedSensorPosition();
    }
}
