package frc.robot.subsystems.arm;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

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
    private boolean homingSecondStep = false;

    public Wrist(
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

        controller.setTolerance(0.05);
    }

    public Wrist(TrapezoidProfile.Constraints constraints, DoubleSupplier pivotAngleSupplier, double range) {
        this(constraints, pivotAngleSupplier, range, 0.0);
    }

    public Wrist(TrapezoidProfile.Constraints constraints, DoubleSupplier pivotAngleSupplier) {
        this(constraints, pivotAngleSupplier, 0.01);
    }

    @Override
    public double getPosition() {
        return motor.getPosition().getValueAsDouble() * 2 * Math.PI * WristConstants.gearRatio;
    }

    @Override
    public double getVelocity() {
        return motor.getVelocity().getValueAsDouble() * 2 * Math.PI * WristConstants.gearRatio;
    }

    @Override
    public void setBrakeMode(boolean brake) {
        motor.setControl(brake ? new StaticBrake() : new CoastOut());
    }

    @Override
    public void jogSetpointPositive() {
        //No clamping because the home is very unreliable
        setpoint += Math.PI / 24;
    }

    @Override
    public void jogSetpointNegative() {
        //No clamping because the home is very unreliable
        setpoint -= Math.PI / 24;
    }

    @Override
    public void home() {
        homing = true;
        setOutput(2);

        homingSecondStep = false;

        homeTimer.reset();
        homeTimer.start();
    }

    @Override
    protected boolean runHomingLogic() {
        /*
         * The homing logic runs in two steps:
         * Step 1: Run the motor until it experiences resistence, assumed to be from the arm
         * Step 2: Wait a little bit for the compliant wheels to squish back, ensuring an accurate reading
         */
        if (!homingSecondStep) {
            if (Math.abs(motor.getStatorCurrent().getValueAsDouble()) < 60) {
                homeTimer.reset();
            } else if (homeTimer.hasElapsed(0.2)) {
                homeTimer.reset();
                homingSecondStep = true;
            }
        } else {
            if (homeTimer.hasElapsed(0.2)) {
                resetOffset();

                homing = false;
                return true;
            }
        }
        return false;
    }

    private void resetOffset() {
        motor.setPosition(
            WristConstants.homedPosition / (2 * Math.PI) / WristConstants.gearRatio);
    }

    @Override
    protected double calculateControl(TrapezoidProfile.State setpoint) {
        double adjustedPosition = getPosition() + pivotAngleSupplier.getAsDouble();

        // TODO: Investigate the merits of having separate PID constants for when the wrist isn't moving
        return controller.calculate(adjustedPosition, setpoint.position)
            + feedforward.calculate(setpoint.position, setpoint.velocity);
    }

    @Override
    public void setOutput(double volts) {
        motor.setControl(new VoltageOut(volts));
    }

    @Override
    public void resetControlLoop() {
        controller.reset();
    }
}
