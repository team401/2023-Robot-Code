package frc.robot.commands.pivot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ArmManager;
import frc.robot.subsystems.pivot.PivotSubsystem;

/**
 * Command that moves the pivot from its current location to a new setpoint.<n>
 * Uses a trapizoidal motion profile, and ends when the profile completes.
 */
public class MovePivot extends CommandBase{
    private PivotSubsystem pivot;

    private TrapezoidProfile profile;
    private State goalState; 
    private double posRad;

    private final Timer timer = new Timer();

    private final Timer finishedTimer = new Timer();

    private final boolean ignoreValidation;

    /**
     * Constructs an instance of this command. The position should be field-
     * relative to the front of the robot.
     * @param pivot the pivot subsystem
     * @param posRad the setpoint position of the pivot in radians. Again,
     * this should be field-relative to the front.
     * @param velRadS the setpoint velocity.
     */
    public MovePivot(PivotSubsystem pivot, double posRad, boolean ignoreValidation) {
        this.pivot = pivot;
        this.posRad = posRad;
        this.ignoreValidation = ignoreValidation;

        addRequirements(this.pivot);
    }

    public MovePivot(PivotSubsystem pivot, double posRad) {
        this(pivot, posRad, false);
    }

    @Override
    public void initialize() {
        finishedTimer.reset();
        finishedTimer.start();

        goalState = new TrapezoidProfile.State(posRad, 0);
        
        // Shift the setpoint to the back of the robot if the pivot is flagged
        // as such.
        if (ArmManager.getInstance().atBack())
            goalState.position = Math.PI - goalState.position;

        // Create the trapezoid motion based on max vel and accel,
        // as well as the current starting state
        timer.reset();
        timer.start();
        // Constraints constraints = DriverStation.isAutonomous() ? new TrapezoidProfile.Constraints(720, 180) : pivot.getConstraintsRad();
        profile = new TrapezoidProfile(pivot.getConstraintsRad(), goalState,
            new State(pivot.getPositionRad(), pivot.getVelRadS()));

        // Marks setpoint in pivot subsystem for the hold command
        pivot.setDesiredSetpointRad(goalState);

        pivot.resetPID();

        pivot.atGoal = false;
    }

    @Override
    public void execute() {
        State setpoint = profile.calculate(timer.get());

        SmartDashboard.putNumber("Pivot/profiled setpoint", setpoint.position);

        SmartDashboard.putNumber("Pivot/final setpoint", setpoint.position);

        double pivotOut = pivot.calculateControl(setpoint, 0);
        pivot.setVolts(pivotOut);

        if (Math.abs(pivot.getPositionRad()-goalState.position) > Units.degreesToRadians(4)) {
            finishedTimer.reset();
        }
    }

    @Override
    public boolean isFinished() {
        return finishedTimer.hasElapsed(0.2) || (ignoreValidation && profile.isFinished(timer.get()));
    }

    @Override
    public void end(boolean interrupted) {
        pivot.stop();
        pivot.atGoal = true;
    }
}