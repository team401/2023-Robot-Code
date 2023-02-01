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

public class MovePivot extends CommandBase{
    private PivotSubsystem pivot;
    // private TelescopeSubsystem telescope;

    private TrapezoidProfile profile;
    private State goalState; 
    private double posRad;
    private double velRadS;

    public double time;

    public MovePivot(PivotSubsystem pivot, /*TelescopeSubsystem telescope,*/ double posRad, 
    double velRadS) {
        this.pivot = pivot;
        // this.telescope = telescope;
        this.posRad = posRad;
        this.velRadS = velRadS;

        // addRequirements(this.pivot);
    }

    public MovePivot(PivotSubsystem pivot, /*TelescopeSubsystem telescope,*/ double posRad) {
        this(pivot, /*telescope,*/ posRad, 0);
    } 

    @Override
    public void initialize() {
        // Create the trapezoid motion in .02s intervals based on max vel and accel,
        // as well as the current starting state
        goalState = new TrapezoidProfile.State(posRad, velRadS);
        
        if (pivot.atBack) {
            goalState.position = Math.PI - goalState.position;
        }

        time = 0;
        profile = new TrapezoidProfile(pivot.getConstraintsRad(), goalState,
            new State(pivot.getPositionRad(), pivot.getVelRadS()));

        SmartDashboard.putNumber("Real Pivot time", profile.totalTime());

        pivot.setDesiredSetpointRad(goalState);

        pivot.resetPID();
    }

    @Override
    public void execute() {
        // Profile should be sampled every .02 seconds, with 0 being the beginning of the profile
        State setpoint = profile.calculate(time);
        time += 0.02;

        SmartDashboard.putNumber("GOING", System.currentTimeMillis());

        // Calculate output from feedforward & PID
        // double pivotOut = pivot.calculateControl(setpoint, 0);
        double pivotOut = pivot.controller.calculate(pivot.getPositionRad(), setpoint.position);
        SmartDashboard.putNumber("pivotOut", pivotOut);
        pivot.setVolts(pivotOut);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}