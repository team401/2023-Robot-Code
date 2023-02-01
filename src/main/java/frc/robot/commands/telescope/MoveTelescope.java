package frc.robot.commands.telescope;


import org.ejml.dense.row.linsol.AdjustableLinearSolver_DDRM;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TelescopeConstants;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

public class MoveTelescope extends CommandBase {
    private TelescopeSubsystem telescope;
    private PivotSubsystem pivot;

    private TrapezoidProfile profile;

    private TelescopeHelper helper;

    private State goalState; 

    private State pivotGoal;

    private double posM;
    private double pivotPosRad;

    private Timer timer = new Timer();


    public MoveTelescope(TelescopeSubsystem telescope,
        PivotSubsystem pivot, double posM, double pivotPosRad) {

        this.telescope = telescope;
        this.pivot = pivot;
        this.posM = posM;
        this.pivotPosRad = pivotPosRad;


        addRequirements(telescope);
    }

    @Override
    public void initialize() {

        timer.reset();
        timer.start();

        goalState = new State(posM, 0);
        pivotGoal = new State(pivotPosRad, 0);

        State holdState = new State(0.05, 0);

        if (pivot.atBack) {
            pivotGoal.position = Math.PI - pivotGoal.position;
        }
       
        TrapezoidProfile pivotProfile = new TrapezoidProfile(
            pivot.getConstraintsRad(),
            pivotGoal,
            new State(pivot.getPositionRad(), pivot.getVelRadS()));
        

        SmartDashboard.putNumber("Fake Pivot Time", pivotProfile.totalTime());

        State currentState = new State(telescope.getPositionM(), telescope.getVel());


        helper = new TelescopeHelper(
            currentState,
            holdState,
            goalState,
            telescope.getConstraints(),
            pivotProfile.totalTime());

        telescope.updateDesiredSetpoint(goalState);
        telescope.resetPID();

        SmartDashboard.putBoolean("Reached Actual Profile", false);
    }

    @Override
    public void execute() {

        State setpoint = helper.calculate(timer.get());
        SmartDashboard.putNumber("MoveTime", timer.get());

        double output = telescope.calculateControl(setpoint, pivot.getPositionRad());

        telescope.setVolts(output);

        SmartDashboard.putNumber("TelescopeSetpoint", setpoint.position);

        SmartDashboard.putNumber("TelescopeVelocity", telescope.getVel());
        SmartDashboard.putNumber("TelescopeSetpointVelocity", setpoint.velocity);
    }

    @Override
    public boolean isFinished() {
        return helper.isFinished(timer.get());
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putNumber("Finishing", System.currentTimeMillis());

    }
}
