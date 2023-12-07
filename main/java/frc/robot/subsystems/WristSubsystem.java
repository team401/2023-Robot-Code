// package frc.robot.subsystems;

// import com.ctre.phoenix6.configs.FeedbackConfigs;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.ControlModeValue;
// import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;

// import edu.wpi.first.math.controller.ArmFeedforward;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Robot;
// import frc.robot.RobotState;
// import frc.robot.Constants.CANDevices;
// import frc.robot.Constants.WristConstants;

// public class WristSubsystem extends ArmSubsystem {
//     private TalonFX motor = new TalonFX(CANDevices.wristMotorID);

//     public boolean homed = false;

//     public boolean atGoal = false;

//     private boolean dead = false;

//     // The subsystem holds its own PID and feedforward controllers and provides calculations from
//     // them, but cannot actually set its own motor output, as accurate feedforward calculations
//     // require information from the pivot subsytem.
//     private final TrapezoidProfile.Constraints constraintsRad = new TrapezoidProfile.Constraints(
//         Units.degreesToRadians(630),
//         Units.degreesToRadians(810));
//     private final PIDController controller = new PIDController(WristConstants.kP, WristConstants.kI, WristConstants.kD);
//     private final PIDController controllerHold = new PIDController(WristConstants.kPHold, WristConstants.kIHold, WristConstants.kDHold);
//     private final ArmFeedforward feedforward = new ArmFeedforward(
//         WristConstants.kS,
//         WristConstants.kG,
//         WristConstants.kV,
//         WristConstants.kA);

//     // Stores the most recent setpoint to allow the Hold command to hold it in place
//     private TrapezoidProfile.State currentSetpointRad = new TrapezoidProfile.State();

//     private double simPos = 0.0;

//     public WristSubsystem() {
//         motor.setInverted(false);

//         motor.setNeutralMode(NeutralModeValue.Brake);

//         motor.configNeutralDeadband(0.004);
        
//         motor.configStatorCurrentLimit(
//             new StatorCurrentLimitConfiguration(true, 70, 80, 0.5));

//         controller.setTolerance(0.05);
//         controllerHold.setTolerance(0.05);
//     }

//     public double getPositionRad() {
//         if (Robot.isReal()) {
//             return motor.getPosition() / 2048 * 2 * Math.PI * WristConstants.gearRatio;
//         }
//         return simPos;
//     }

//     public double getVelRadS() {
//         return motor.getSelectedSensorVelocity() / 2048 * 2 * Math.PI * 10 * WristConstants.gearRatio;
//     }

//     public double getAmps() {
//         return Math.abs(motor.getStatorCurrent().getValueAsDouble());
//     }

//     public void setBrakeMode(boolean braked) {
//         motor.setNeutralMode(braked ? NeutralModeValue.Brake : NeutralModeValue.Coast);
//     }

//     /**
//      * @return Most recent setpoint set by a move command. This setpoint only exists for utility
//      * purposes and is not used by the subsystem
//      */
//     public TrapezoidProfile.State getDesiredSetpointRad() {
//         return currentSetpointRad;
//     }

//     public void jogSetpointForward() {
//         currentSetpointRad.position += Math.PI / 24;
//     }

//     public void jogSetpointBack() {
//         currentSetpointRad.position -= Math.PI / 24;
//     }
    
//     public TrapezoidProfile.Constraints getConstraintsRad() {
//         return constraintsRad;
//     }

//     public void toggleKill() {
//         motor.set(0);
//         dead = !dead;
//     }

//     public void setCurrentLimit(double currentLimit, double triggerThresholdCurrent, double triggerThresholdTime) {
//         motor.configStatorCurrentLimit(
//             new StatorCurrentLimitConfiguration(true, currentLimit, triggerThresholdCurrent, triggerThresholdTime));

//     }

//     /**
//      * Does control calculations from its ArmFeedforward and PID controllers.
//      * Does not command any motors.
//      * @param setpoint The given setpoint, field-relative
//      * @param angleDeg The angle of the wrist, also field-relative. Do not give
//      * direct sensor values
//      * @return The result of the calculation
//      */
//     public double calculateControl(TrapezoidProfile.State setpointRad, double angleRad, boolean holding) {

//         double fb = holding ? controllerHold.calculate(angleRad, setpointRad.position) : controller.calculate(angleRad, setpointRad.position);
//         double ff = feedforward.calculate(setpointRad.position, setpointRad.velocity);

//         return fb + ff;
        
//     }

//     public double calculateControl(TrapezoidProfile.State setpointRad, double angleRad, boolean holding, double pivotVel) {

//         double fb = holding ? controllerHold.calculate(angleRad, setpointRad.position) : controller.calculate(angleRad, setpointRad.position);
//         double ff = feedforward.calculate(setpointRad.position, setpointRad.velocity + pivotVel);

//         return fb + ff;
        
//     }

//     /**
//      * Resets the PID controller stored in this subsystem.
//      */
//     public void resetPID() {
//         controller.reset();
//         controllerHold.reset();
//     }

//     /**
//      * @param state The setpoint state the wrist should be driven to.
//      * Has no effect on the function of this subsytem.
//      */
//     public void updateDesiredSetpointRad(TrapezoidProfile.State state) {
//         currentSetpointRad = state;
//     }

//     public void setSimPosRad(double pos) {
//         simPos = pos;
//     }

//     /**
//      * For homing. Resets the encoder offset to the position of 141 degrees,
//      * the position the wrist should be at when it goes all the way to the arm.
//      */
//     public void resetOffset() {
//         motor.setSelectedSensorPosition(2.81 / (2 * Math.PI) * 2048 / WristConstants.gearRatio);
//     }

//     public void resetOffsetCube() {
//         motor.setSelectedSensorPosition(1.5 / (2 * Math.PI) * 2048 / WristConstants.gearRatio);
//     }

//     public void zeroOffset() {
//         motor.setSelectedSensorPosition(0);
//     }

//     public void setVolts(double input) {
//         if (!dead) {
//             motor.set(input / 12);
//             return;
//         }
//         motor.set(0);
//     }

//     public void overrideVolts(double input) {
//         System.out.println(input / 12);
//         motor.set(input / 12);
//     }

//     public void stop() {
//         motor.set(0);
//     }

//     public void periodic() {
//         SmartDashboard.putNumber("Wrist/position", getPositionRad());
//         SmartDashboard.putNumber("Wrist/motor current", getAmps());

//         SmartDashboard.putBoolean("Wrist/dead", dead);
//         // SmartDashboard.putNumber("Wrist/applied voltage", motor.getMotorOutputVoltage());
//         SmartDashboard.putNumber("Wrist/velocity", getVelRadS());

//         RobotState.getInstance().putWristDisplay(getPositionRad());

//     }
// }