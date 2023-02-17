package frc.robot.commands.pivot;


import java.util.function.DoubleSupplier;

import org.opencv.core.Mat;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotState;
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
    private DoubleSupplier posRad;

    public Timer timer = new Timer();

    /**
     * Constructs an instance of this command. The position should be field-
     * relative to the front of the robot.
     * @param pivot the pivot subsystem
     * @param telescope the telescope subsystem
     * @param posRad the setpoint position of the pivot in radians. Again,
     * this should be field-relative to the front.
     * @param velRadS the setpoint velocity.
     */
    public MovePivot(PivotSubsystem pivot, TelescopeSubsystem telescope, DoubleSupplier posRad) {
        this.pivot = pivot;
        this.telescope = telescope;
        this.posRad = posRad;

        addRequirements(this.pivot);
    }

    @Override
    public void initialize() {
        goalState = new TrapezoidProfile.State(posRad.getAsDouble(), 0);
        
        // Shift the setpoint to the back of the robot if the pivot is flagged
        // as such.
        if (RobotState.getInstance().atBack()) {
            goalState.position = Math.PI - goalState.position;
        }

        // Create the trapezoid motion based on max vel and accel,
        // as well as the current starting state
        timer.reset();
        timer.start();
        profile = new TrapezoidProfile(pivot.getConstraintsRad(), goalState,
            new State(pivot.getPositionRad(), pivot.getVelRadS()));

        // Marks setpoint in pivot subsystem for the hold command
        pivot.setDesiredSetpointRad(goalState);

        pivot.resetPID();
    }

    @Override
    public void execute() {
        State setpoint = profile.calculate(timer.get());

        SmartDashboard.putNumber("MovePivot State", setpoint.position);

        //TODO: change to actual feedforward
        // Calculate output from feedforward & PID
        double pivotOut = pivot.calculateControl(setpoint, 0);
        // double pivotOut = pivot.controller.calculate(pivot.getPositionRad(), setpoint.position);
        // SmartDashboard.putNumber("pivotOut", pivotOut);
        pivot.setVolts(pivotOut);
        // pivot.setSimPos(setpoint.position);
    }

    @Override
    public boolean isFinished() {
        return profile.isFinished(timer.get());
        // return false;
    }

    @Override
    public void end(boolean interrupted) {
        pivot.stop();
    }
}