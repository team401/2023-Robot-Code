package frc.robot.subsystems.arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public abstract class GenericArmJoint {

    protected double setpoint;
    protected boolean homing = false;

    protected boolean active = true;

    protected TrapezoidProfile.Constraints constraints;

    public GenericArmJoint(TrapezoidProfile.Constraints constraints, double defaultSetpoint) {
        this.constraints = constraints;
        setpoint = defaultSetpoint;
    }

    public GenericArmJoint(TrapezoidProfile.Constraints constraints) {
        this(constraints, 0.0);
    }

    public void home() {
        throw new UnsupportedOperationException(
                "This joint cannot home. If it should be able to home, implement `home()`");
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
        resetControlLoop();
    }

    public void runControls() {
        if (!active) {
            setInput(0.0);
        }

        if (homing) {
            // It's expected that homing logic could be different enough from joint to joint
            // to require a custom implementation.
            runHomingLogic();
        }

        TrapezoidProfile.State goalState = new TrapezoidProfile.State(setpoint, 0);
        TrapezoidProfile.State currentState = new TrapezoidProfile.State(getPosition(), getVelocity());

        TrapezoidProfile profile = new TrapezoidProfile(constraints, goalState, currentState);

        double input = calculateControlInput(profile.calculate(Constants.loopTime));
        setInput(input);
    }

    /**
     * Set the control input to this joint, in volts
     */
    protected abstract void setInput(double volts);

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
    protected abstract double calculateControlInput(TrapezoidProfile.State setpointState);

    /**
     * Called every time the setpoint position is updated.<p>
     * Intended to be used to avoid PID integral windup amoung other strange effects.
     */
    protected void resetControlLoop() {}

    /**
     * Returns the current absolute position of this joint
     */
    // TODO: upgrade to 2024 and try out the units library
    public abstract double getPosition();

    /**
     * Returns the current velocity of this joint in units / second
     */
    public abstract double getVelocity();

    public boolean isActive() {
        return active;
    }
}
