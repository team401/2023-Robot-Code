package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.TelescopeConstants;

public class TelescopeSubsystem extends SubsystemBase{
    private TalonFX motor = new TalonFX(CANDevices.telescopeMotorID);

    public boolean homed = false;

    // For safety; detect when encoder stops sending new data
    private double lastEncoderPos;
    private boolean dead = false;

    private double simPos;

    // The subsystem holds its own PID and feedforward controllers and provides calculations from
    // them, but cannot actually set its own motor output, as accurate feedforward calculations
    // require information from the pivot subsytem.
    private final PIDController controller = new PIDController(TelescopeConstants.kP, 0, 0);
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
        TelescopeConstants.kS,
        TelescopeConstants.kV);
    private final TrapezoidProfile.Constraints constraints = 
        new TrapezoidProfile.Constraints(4, 4);

    // Stores the most recent setpoint to allow the Hold command to hold it in place
    private TrapezoidProfile.State currentSetpoint = new TrapezoidProfile.State(0.06, 0);

    public TelescopeSubsystem() {
        motor.setInverted(InvertType.None);
        motor.setNeutralMode(NeutralMode.Brake);
        
        motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        motor.setSensorPhase(false);

        motor.configNeutralDeadband(0.004);

        motor.configStatorCurrentLimit(
            new StatorCurrentLimitConfiguration(true, 40, 50, 0.5));

        SmartDashboard.putNumber("Telescope test setpoint", 0);
    }

    public double getPositionM() {
        // 4096 units per rotation, multiply rotations by diameter
        return motor.getSelectedSensorPosition() / 4096
            * 2 * Math.PI  * TelescopeConstants.conversionM;//* TelescopeConstants.wheelRadiusM
            //* TelescopeConstants.gearRatio;

        // return simPos;
    }

    public double getVel() {
        return motor.getSelectedSensorVelocity() / 4096
            * 2 * Math.PI * TelescopeConstants.conversionM * 10;
    }

    public double getAmps() {
        return Math.abs(motor.getStatorCurrent());
    }

     /**
     * @return Most recent setpoint set by a move command. This setpoint only exists for utility
     * purposes and is not used by the subsystem
     */
    public TrapezoidProfile.State getDesiredSetpoint() {
        return currentSetpoint;
    }

    public TrapezoidProfile.Constraints getConstraints() {
        return constraints;
    }

    /**
     * Does control calculations from its ArmFeedforward and PID controllers.
     * Does not command any motors.
     * @param setpoint The given setpoint
     * @param pivotAngleRad The position of the pivot, required for accurate feedforward
     * @return The result of the calculation. Add kG * sine of the pivot angle
     */
    public double calculateControl(TrapezoidProfile.State setpoint, double pivotAngleRad) {
        SmartDashboard.putNumber("Telescope calculated", controller.calculate(getPositionM(), setpoint.position));
        return controller.calculate(getPositionM(), setpoint.position)
            + feedforward.calculate(setpoint.velocity)
            // Compensates for weight of telescope as the pivot goes up
            // Gravity Constant * Sine of Angle
            + TelescopeConstants.kG * Math.sin(pivotAngleRad);
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
    public void setDesiredSetpoint(TrapezoidProfile.State state) {
        currentSetpoint = state;
    }

    public void jogSetpointForward() {
        currentSetpoint.position += 0.05;
    }

    public void jogSetpointBackward() {
        currentSetpoint.position -= 0.05;
    }

    public void setSimPos(double pos) {
        simPos = pos;
    }

    /**
     * @return a new InstantCommand that stops the motor and requires this subsystem
     */
    public Command killCommand() {
        return new InstantCommand(this::die, this);
    }

    public void setVolts(double input) {
        if (!dead)
            motor.set(ControlMode.PercentOutput, input / 12);
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

    public void resetOffset() {
        motor.setSelectedSensorPosition(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Telescope Position", getPositionM()); 
        SmartDashboard.putNumber("Telescope Velocity", getVel());
        SmartDashboard.putNumber("Telescope Desired Position", currentSetpoint.position);
        SmartDashboard.putBoolean("Telescope Dead", dead);
        SmartDashboard.putNumber("Telescope Voltage", motor.getMotorOutputVoltage());
        SmartDashboard.putNumber("Telescope Amps", getAmps());

        RobotState.getInstance().putTelescopeDisplay(getPositionM());
    }
}
