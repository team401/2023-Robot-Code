package frc.robot.subsystems.arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;

public abstract class GenericArmJoint {

    protected double setpoint;
    protected boolean homing = false;

    protected boolean active = true;

    protected double range;

    protected TrapezoidProfile motionProfile;
    protected Timer motionTimer = new Timer();

    public GenericArmJoint(TrapezoidProfile.Constraints constraints, double range, double defaultSetpoint) {
        this.motionProfile = new TrapezoidProfile(constraints);
        this.range = range;
        setpoint = defaultSetpoint;
        motionTimer.start();
    }

    public GenericArmJoint(TrapezoidProfile.Constraints constraints, double range) {
        this(constraints, range, 0.0);
    }

    /**
     * Signal this joint to home.
     */
    public void home() {
        throw new UnsupportedOperationException(
                "This joint cannot home. If it should be able to home, implement `home()`");
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
        motionTimer.reset();
        resetControlLoop();
    }

    public void runControls() {
        update();

        if (!active) {
            setOutput(0.0);
        }

        if (homing) {
            // It's expected that homing logic could be different enough from joint to joint
            // to require a custom implementation.
            runHomingLogic();
        }

        TrapezoidProfile.State goalState = new TrapezoidProfile.State(setpoint, 0);
        TrapezoidProfile.State currentState = new TrapezoidProfile.State(getPosition(), getVelocity());

        double output = calculateControl(
                motionProfile.calculate(motionTimer.get(), goalState, currentState));
        setOutput(output);
    }

    public abstract void jogSetpointPositive();

    public abstract void jogSetpointNegative();

    public abstract void setBrakeMode(boolean brake);

    /**
     * Optional Override: Update internal state periodically
     */
    protected void update() {
    }

    /**
     * Set the control input to this joint, in volts
     */
    protected abstract void setOutput(double volts);

    /**
     * Run a complete iteration of the homing sequence.
     * 
     * @return Whether this joint is finished homing
     */
    protected boolean runHomingLogic() {
        // throw exception like `home()`?
        return true;
    }

    /**
     * Calculates the control input, in volts, required to reach the desired
     * position and velocity based on feedforward and feedback control algorthms.
     * 
     * @param setpointState Desired position and velocity based on motion profile
     * @return system input voltage
     */
    protected abstract double calculateControl(TrapezoidProfile.State setpointState);

    /**
     * Called every time the setpoint position is updated.
     * <p>
     * Intended to be used to avoid PID integral windup amoung other strange
     * effects.
     */
    protected void resetControlLoop() {
    }

    /**
     * Returns the current absolute position of this joint
     */
    // TODO: upgrade to 2024 and try out the units library
    public abstract double getPosition();

    /**
     * Whether the joint's position is within range of its setpoint.
     */
    public boolean atSetpoint() {
        return setpoint - range / 2 < getPosition() && getPosition() < setpoint + range / 2;
    }

    /**
     * Returns the current velocity of this joint in units / second
     */
    public abstract double getVelocity();

    /**
     * If the joint is inactive, it will not provide power to its motors.
     * 
     * @return
     */
    public boolean isActive() {
        return active;
    }

    /**
     * If this joint is active, it is 'alive,' and will attempt to drive itself to
     * its setpoint
     * using a control loop. If it is not, it is 'dead,' and will never apply output
     * power.
     */
    public void setActive(boolean active) {
        this.active = active;
    }
}
