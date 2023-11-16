package frc.robot.subsystems.arm;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.TelescopeConstants;

public class Telescope extends GenericArmJoint {
    
    private TalonFX motor = new TalonFX(CANDevices.telescopeMotorID);
    
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
        TelescopeConstants.kS,
        TelescopeConstants.kV);
    private final PIDController controller = 
        new PIDController(TelescopeConstants.kP, 0, 0);

    private final DoubleSupplier pivotPositionSupplier;


    public Telescope(
        TrapezoidProfile.Constraints constraints,
        DoubleSupplier pivotPositionSupplier,
        double defaultSetpoint
    ) {
        super(constraints, defaultSetpoint);

        this.pivotPositionSupplier = pivotPositionSupplier;

        motor.setInverted(InvertType.None);
        motor.setNeutralMode(NeutralMode.Brake);
        
        motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        motor.setSensorPhase(false);

        motor.configNeutralDeadband(0.004);

        motor.configStatorCurrentLimit(
            new StatorCurrentLimitConfiguration(true, 40, 50, 0.5));
    }

    public Telescope(TrapezoidProfile.Constraints constraints, DoubleSupplier pivotPositionSupplier) {
        this(constraints, pivotPositionSupplier, TelescopeConstants.stowedPosition);
    }

    @Override
    public double getPosition() {
        return motor.getSelectedSensorPosition() / 4096
            * 2 * Math.PI  * TelescopeConstants.conversionM;
    }

    @Override
    public double getVelocity() {
        return motor.getSelectedSensorVelocity() / 4096
            * 2 * Math.PI * TelescopeConstants.conversionM * 10;
    }

    @Override
    protected double calculateControlInput(TrapezoidProfile.State setpoint) {
        return controller.calculate(getPosition(), setpoint.position)
            + feedforward.calculate(setpoint.velocity)
            // Compensates for weight of telescope as the pivot goes up
            // Gravity Constant * Sine of Angle
            + TelescopeConstants.kG * Math.sin(pivotPositionSupplier.getAsDouble());
    }

    @Override
    protected void setInput(double volts) {
        motor.set(ControlMode.PercentOutput, volts / 12);
    }
}
