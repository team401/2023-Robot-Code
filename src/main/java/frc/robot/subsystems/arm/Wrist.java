package frc.robot.subsystems.arm;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.WristConstants;

public class Wrist extends GenericArmJoint {

    private TalonFX motor = new TalonFX(CANDevices.wristMotorID);
    
    private final PIDController controller = 
        new PIDController(WristConstants.kP, WristConstants.kI, WristConstants.kD);
    private final ArmFeedforward feedforward = new ArmFeedforward(
        WristConstants.kS,
        WristConstants.kG,
        WristConstants.kV,
        WristConstants.kA);

    private final DoubleSupplier pivotAngleSupplier;

    private final Timer homeTimer = new Timer();

    public Wrist(
        TrapezoidProfile.Constraints constraints,
        DoubleSupplier pivotAngleSupplier,
        double defaultSetpoint
    ) {
        super(constraints, defaultSetpoint);

        this.pivotAngleSupplier = pivotAngleSupplier;

        motor.setInverted(false);
        motor.setInverted(InvertType.None);

        motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        motor.setNeutralMode(NeutralMode.Brake);

        motor.configNeutralDeadband(0.004);
        
        motor.configStatorCurrentLimit(
            new StatorCurrentLimitConfiguration(true, 70, 80, 0.5));

        controller.setTolerance(0.05);
    }

    public Wrist(TrapezoidProfile.Constraints constraints, DoubleSupplier pivotAngleSupplier) {
        this(constraints, pivotAngleSupplier, 0.0);
    }

    @Override
    public double getPosition() {
        return motor.getSelectedSensorPosition() / 2048 * 2 * Math.PI * WristConstants.gearRatio;
    }

    @Override
    public double getVelocity() {
        return motor.getSelectedSensorVelocity() / 2048 * 2 * Math.PI * 10 * WristConstants.gearRatio;
    }

    @Override
    public void home() {
        homing = true;
        setOutput(2);

        homeTimer.reset();
        homeTimer.start();
    }

    @Override
    protected boolean runHomingLogic() {
        if (Math.abs(motor.getStatorCurrent()) < 60) {
            // homeTimer.reset
        }
        return true;
    }

    @Override
    protected double calculateControl(TrapezoidProfile.State setpoint) {
        
    }

    @Override
    public void setOutput(double volts) {
        motor.set(ControlMode.PercentOutput, volts);
    }
    
}
