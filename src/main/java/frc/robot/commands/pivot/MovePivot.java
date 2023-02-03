package frc.robot.commands.pivot;


import org.opencv.core.Mat;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.TelescopeConstants;
import frc.robot.subsystems.PivotSubsystem;
// import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

/**
 * Command that moves the pivot from its current location to a new setpoint.<n>
 * Uses a trapizoidal motion profile, and ends when the profile completes.
 */
public class MovePivot extends CommandBase{
    private PivotSubsystem pivot;
    private TelescopeSubsystem telescope;

    private TrapezoidProfile profile;
    private State goalState; 
    private double posRad;
    private double velRadS;

    //TODO: Replace with timer to appease Sullivan
    public double time;

    /**
     * Constructs an instance of this command. The position should be field-
     * relative to the front of the robot.
     * @param pivot the pivot subsystem
     * @param telescope the telescope subsystem
     * @param posRad the setpoint position of the pivot in radians. Again,
     * this should be field-relative to the front.
     * @param velRadS the setpoint velocity.
     */
    public MovePivot(PivotSubsystem pivot, TelescopeSubsystem telescope, double posRad, 
    double velRadS) {
        this.pivot = pivot;
        // this.telescope = telescope;
        this.posRad = posRad;
        this.velRadS = velRadS;

        // addRequirements(this.pivot);
    }

    /**
     * Constructs an instance of this command. The position should be field-
     * relative to the front of the robot.
     * @param pivot the pivot subsystem
     * @param telescope the telescope subsystem
     * @param posRad the setpoint position of the pivot in radians. Again,
     * this should be field-relative to the front.
     */
    public MovePivot(PivotSubsystem pivot, TelescopeSubsystem telescope, double posRad) {
        this(pivot, telescope, posRad, 0);
    } 

    @Override
    public void initialize() {
        goalState = new TrapezoidProfile.State(posRad, velRadS);
        
        // Shift the setpoint to the back of the robot if the pivot is flagged
        // as such.
        if (pivot.atBack) {
            goalState.position = Math.PI - goalState.position;
        }

        // Create the trapezoid motion in .02s intervals based on max vel and accel,
        // as well as the current starting state
        time = 0;
        profile = new TrapezoidProfile(pivot.getConstraintsRad(), goalState,
            new State(pivot.getPositionRad(), pivot.getVelRadS()));

        SmartDashboard.putNumber("Real Pivot time", profile.totalTime());

        // Marks setpoint in pivot subsystem for the hold command
        pivot.setDesiredSetpointRad(goalState);

        pivot.resetPID();
    }

    @Override
    public void execute() {
        // Profile should be sampled every .02 seconds, with 0 being the beginning of the profile
        State setpoint = profile.calculate(time);
        time += 0.02;

        SmartDashboard.putNumber("GOING", System.currentTimeMillis());

        //TODO: change to actual feedforward
        // Calculate output from feedforward & PID
        // double pivotOut = pivot.calculateControl(setpoint, 0);
        double pivotOut = pivot.controller.calculate(pivot.getPositionRad(), setpoint.position);
        SmartDashboard.putNumber("pivotOut", pivotOut);
        pivot.setVolts(pivotOut);
    }

    @Override
    public boolean isFinished() {
        //TODO: return profile.isFinished()
        return false;
    }
}