package frc.robot.commands.telescope;


import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

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
import frc.robot.RobotState;
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

    private DoubleSupplier goalM;
    private DoubleSupplier pivotGoalRad;

    private Timer timer = new Timer();


    public MoveTelescope(
        TelescopeSubsystem telescope,
        PivotSubsystem pivot,
        DoubleSupplier goalM,
        DoubleSupplier pivotGoalRad) {
        
        this.goalM = goalM;
        this.pivotGoalRad = pivotGoalRad;
        this.telescope = telescope;
        this.pivot = pivot;


        addRequirements(telescope);
    }

    @Override
    public void initialize() {

        timer.reset();
        timer.start();

        goalState = new State(goalM.getAsDouble(), 0);
        pivotGoal = new State(pivotGoalRad.getAsDouble(), 0);

        State holdState = new State(0.05, 0);

        if (RobotState.getInstance().atBack())
            pivotGoal.position = Math.PI - pivotGoal.position;
       
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

        telescope.setDesiredSetpoint(goalState);
        telescope.resetPID();
    }

    @Override
    public void execute() {

        State setpoint = helper.calculate(timer.get());

        double output = telescope.calculateControl(setpoint, pivot.getPositionRad());

        SmartDashboard.putNumber("Telescope Setpoint", setpoint.position);

        telescope.setVolts(output);

        telescope.setSimPos(setpoint.position);
    }

    @Override
    public boolean isFinished() {
        return helper.isFinished(timer.get());
    }
}
