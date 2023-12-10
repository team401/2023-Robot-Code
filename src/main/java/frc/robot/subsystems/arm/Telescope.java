package frc.robot.subsystems.arm;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.TelescopeConstants;

public class Telescope extends GenericArmJoint {
    
    private TalonFX motor = new TalonFX(CANDevices.telescopeMotorID);
    
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
        TelescopeConstants.kS,
        TelescopeConstants.kV);
    private final PIDController feedbackController = 
        new PIDController(TelescopeConstants.kP, 0, 0);

    private final DoubleSupplier pivotAngleSupplier;

    private final Timer homeTimer = new Timer();


    public Telescope(
        TrapezoidProfile.Constraints constraints,
        DoubleSupplier pivotAngleSupplier,
        double range,
        double defaultSetpoint
    ) {
        super(constraints, range, defaultSetpoint);

        this.pivotAngleSupplier = pivotAngleSupplier;

        motor.setInverted(false);
        motor.setControl(new StaticBrake());
        
        //I have no idea how to set a current limit from the API
    }

    public Telescope(TrapezoidProfile.Constraints constraints, DoubleSupplier pivotPositionSupplier, double range) {
        this(constraints, pivotPositionSupplier, range, TelescopeConstants.stowedPosition);
    }

    public Telescope(TrapezoidProfile.Constraints constraints, DoubleSupplier pivotPositionSupplier) {
        this(constraints, pivotPositionSupplier, 0.02);
    }

    @Override
    public double getPosition() {
        return motor.getPosition().getValueAsDouble() * 2 * Math.PI
            * TelescopeConstants.conversionM;
    }

    @Override
    public double getVelocity() {
        return motor.getVelocity().getValueAsDouble() * 2 * Math.PI
            * TelescopeConstants.conversionM;
    }

    @Override
    public void setBrakeMode(boolean brake) {
        motor.setControl(brake ? new StaticBrake() : new CoastOut());
    }

    @Override
    public void jogSetpointPositive() {
        setpoint = MathUtil.clamp(
            setpoint + 0.05,                
            TelescopeConstants.minPosM,
            TelescopeConstants.maxPosM);
    }

    @Override
    public void jogSetpointNegative() {
        setpoint = MathUtil.clamp(
            setpoint - 0.05,                
            TelescopeConstants.minPosM,
            TelescopeConstants.maxPosM);
    }

    @Override
    public void home() {
        homing = true;

        setOutput(-1.0);

        homeTimer.reset();
        homeTimer.start();
    }

    @Override
    protected boolean runHomingLogic() {
        if (motor.getStatorCurrent().getValue() < 30) {
            homeTimer.reset();
        }

        if (homeTimer.hasElapsed(0.3)) {
            motor.setPosition(0);

            homing = false;
            return true;
        }

        return false;
    }

    @Override
    protected double calculateControl(TrapezoidProfile.State setpoint) {
        return feedbackController.calculate(getPosition(), setpoint.position)
            + feedforward.calculate(setpoint.velocity)
            // Compensates for weight of telescope as the pivot goes up
            // Gravity Constant * Sine of Angle
            + TelescopeConstants.kG * Math.sin(pivotAngleSupplier.getAsDouble());
    }

    @Override
    protected void setOutput(double volts) {
        motor.setControl(new VoltageOut(volts));
    }

    @Override
    protected void resetControlLoop() {
        feedbackController.reset();
    }
}
