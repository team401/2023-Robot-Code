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
import frc.robot.Constants.ArmPositions;
import frc.robot.Constants.TelescopeConstants;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

public class MoveTelescope extends CommandBase {
    private TelescopeSubsystem telescope;
    private PivotSubsystem pivot;

    private TrapezoidProfile profile;

    private TelescopeHelper helper;

    private double goalM;

    private double pivotGoalRad;

    private Timer timer = new Timer();

    private Timer finishedTimer = new Timer();


    public MoveTelescope(
        TelescopeSubsystem telescope,
        PivotSubsystem pivot,
        double goalM,
        double pivotGoalRad) {
        
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

        finishedTimer.reset();
        finishedTimer.start();

        State goalState = new State(goalM, 0);
        State pivotGoalState = new State(pivotGoalRad, 0);
        State holdState = new State(ArmPositions.stow[2], 0);
        if (RobotState.getInstance().atBack())
            pivotGoalState.position = Math.PI - pivotGoalState.position;

            State currentState = new State(telescope.getPositionM(), telescope.getVel());
        
        TrapezoidProfile pivotProfile = new TrapezoidProfile(
            pivot.getConstraintsRad(),
            pivotGoalState,
            new State(pivot.getPositionRad(), pivot.getVelRadS())
        );

        profile = new TrapezoidProfile(telescope.getConstraints(), goalState, currentState);

        helper = new TelescopeHelper(
            currentState,
            holdState,
            goalState,
            telescope.getConstraints(),
            pivotProfile.totalTime()
        );

        telescope.setDesiredSetpoint(goalState);
        telescope.resetPID();

        telescope.atGoal = false;
    }

    @Override
    public void execute() {

        State setpoint = helper.calculate(timer.get());

        double output = telescope.calculateControl(setpoint, pivot.getPositionRad());

        SmartDashboard.putNumber("Telescope Setpoint", setpoint.position);

        telescope.setVolts(output);

        telescope.setSimPos(setpoint.position);

        if (Math.abs(telescope.getPositionM()-goalM) > 0.01) {
            finishedTimer.reset();
        }
    }

    @Override
    public boolean isFinished() {
        return finishedTimer.hasElapsed(0.2);
    }

    @Override
    public void end(boolean interrupted) {
        telescope.stop();
        telescope.atGoal = true;
    }
}
