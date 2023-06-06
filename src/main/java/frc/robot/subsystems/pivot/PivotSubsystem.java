package frc.robot.subsystems.pivot;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.TelescopeConstants;

public class PivotSubsystem extends SubsystemBase {
    private final PivotIO io;
    private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

    private boolean dead = false;
    public boolean atGoal = false;

    // The subsystem holds its own PID and feedforward controllers and provides calculations from
    // them, but cannot actually set its own motor output, as accurate feedforward calculations
    // require information from the telescope subsytem.
    private TrapezoidProfile.Constraints constraintsRad = 
        new TrapezoidProfile.Constraints(
            Units.degreesToRadians(360),
            Units.degreesToRadians(600));
    public final PIDController controller = new PIDController(
        PivotConstants.kP, 
        0, 
        PivotConstants.kD);
    private final ArmFeedforward feedforward = new ArmFeedforward(
        PivotConstants.kS,
        PivotConstants.kG,
        PivotConstants.kV,
        PivotConstants.kA);
    

    // Stores the most recent setpoint to allow the Hold command to hold it in place
    private TrapezoidProfile.State currentSetpointRad = 
        new TrapezoidProfile.State(getPositionRad(), 0);


    public PivotSubsystem(PivotIO io) {
        this.io = io;

        // SmartDashboard.putNumber("Pivot test setpoint", 0);
    }

    public double getPositionRad() {
        return inputs.positionRad;
        // return encoder.getAbsolutePosition();
    }

    public double getVelRadS() {
        return inputs.velocityRadPerSec;
    }

    public void toggleKill() {
        io.setVolts(0);
        dead = !dead;
    }

    public void autoConstrain() {
        constraintsRad = 
            new TrapezoidProfile.Constraints(
                Units.degreesToRadians(360),
                Units.degreesToRadians(450));
    }

    public void printAutoTimer() {
        SmartDashboard.putNumber("autoTmp", autoTimer.get());
    }

    public void normalConstrain() {
        constraintsRad = 
            new TrapezoidProfile.Constraints(
                Units.degreesToRadians(360),
                Units.degreesToRadians(600));
    }

    /**
     * @return Most recent setpoint set by a move command. This setpoint only exists for utility
     * purposes and is not used by the subsystem
     */
    public TrapezoidProfile.State getDesiredSetpointRad() {
        return currentSetpointRad;
    }

    public TrapezoidProfile.Constraints getConstraintsRad() {
        return constraintsRad;
    }

    /**
     * 
     * @param setpointRad
     * @param telescopePosM
     * @return
     */
    public double calculateControl(TrapezoidProfile.State setpointRad, double telescopePosM) {

        return controller.calculate(getPositionRad(), setpointRad.position)
            + feedforward.calculate(setpointRad.position, setpointRad.velocity)
            // Compensates for telescope extention
            // Gravity constant * Telescope Extention (proportion) * Cosine of Angle
            + PivotConstants.extraKg * telescopePosM / TelescopeConstants.maxPosMeters
                * Math.cos(getPositionRad());
    }

    /**
     * Resets the PID controller stored in this subsystem.
     */
    public void resetPID() {
        controller.reset();
    }

    /**
     * @param state The setpoint state the telescope should be driven to.
     * Has no effect on the function of this subsytem.
     */
    public void setDesiredSetpointRad(TrapezoidProfile.State stateRad) {
        currentSetpointRad = stateRad;
    }

    public void jogSetpointForward() {
        currentSetpointRad.position = 
            MathUtil.clamp(
                    currentSetpointRad.position + Math.PI/64,
                    PivotConstants.maxFwdRotationRad,
                    PivotConstants.maxBackRotationRad);
    }

    public void jogSetpointBack() {
        currentSetpointRad.position = 
            MathUtil.clamp(
                    currentSetpointRad.position - Math.PI/64,
                    PivotConstants.maxFwdRotationRad,
                    PivotConstants.maxBackRotationRad);
    }
 
    /**
     * Sets the output power of the motors in volts if the boundry conditions
     * are not exceeded. If the pivot is out of bounds or dead, the voltage
     * will be set to 0.
     * @param input The voltage to set
     */
    public void setVolts(double input) {
        if (!dead && withinSoftLimits(input)) {
            io.setVolts(input);
            // SmartDashboard.putNumber("Pivot Commanded V", input);
        } else {
            io.setVolts(0);
        }
    }

    public void stop() {
        io.setVolts(0);
    }

    /**
     * Sets the output power of the motors in volts, even if it is past the max
     * rotations or dead. Voltage is divided in half. <p>
     * 
     * ONLY USED BY HUMANS. Do not have PID or feedforward use this method.
     * @param input The voltage to set
     */
    public void overrideVolts(double input) {
        io.setVolts(input);
    }


    /**
     * 'Kills' the subsystem. The motor will be stopped and no loger respond to input
     */
    public void die() {
        setVolts(0);
        dead = true;
    }

    public void setBrakeMode(boolean braked) {
        io.setBrakeMode(braked);
    }
    
    /**
     * 'Revives' the subsytem. If it is dead, the motor will start responding.<p>
     * DO NOT have regular code call this method. Only a human button should do this.
     */
    public void revive() {
        dead = false;
    }

    /**
     * @return a new InstantCommand that stops the motor and requires this subsystem
     */
    public Command killCommand() {
        return new InstantCommand(this::die, this);
    }



    private Timer autoTimer = new Timer();

    @Override
    public void periodic() {

        SmartDashboard.putNumber("AutoTimer", autoTimer.get());

        // SmartDashboard.putNumber("Pivot Position", getPositionRad());
        // SmartDashboard.putNumber("Pivot Velocity", getVelRadS());
        // SmartDashboard.putNumber("Right Pivot Current", rightMotor.getStatorCurrent());
        // SmartDashboard.putNumber("Left Pivot Current", leftMotor.getStatorCurrent());
        // SmartDashboard.putNumber("Pivot Desired Setpoint", currentSetpointRad.position);

        // SmartDashboard.putBoolean("Pivot Dead", dead);

        // SmartDashboard.putBoolean("At Back", RobotState.getInstance().atBack());

        io.updateInputs(inputs);

        Logger.getInstance().processInputs("Pivot", inputs);

        RobotState.getInstance().putPivotDisplay(getPositionRad());

        //if (DriverStation.isEnabled()) checkIfDead();
    }

    public void startAutoTimer() {
        autoTimer.reset();
        autoTimer.start();
    }

    public void stopAutoTimer() {
        autoTimer.stop();
    }

    private boolean withinSoftLimits(double input) {
        if (input > 0 && getPositionRad() > PivotConstants.maxBackRotationRad) {
            return false;
        }
        if (input < 0 && getPositionRad() < PivotConstants.maxFwdRotationRad) {
            return false;
        }
        return true;
    }
}
