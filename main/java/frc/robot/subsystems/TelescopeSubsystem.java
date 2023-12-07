// package frc.robot.subsystems;

// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.NeutralModeValue;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Robot;
// import frc.robot.RobotState;
// import frc.robot.Constants.CANDevices;
// import frc.robot.Constants.TelescopeConstants;

// public class TelescopeSubsystem extends ArmSubsystem{
//     private TalonFX motor = new TalonFX(CANDevices.telescopeMotorID);

//     public boolean homed = false;

//     private boolean dead = false;

//     public boolean atGoal = false;

//     private double simPos;

//     // The subsystem holds its own PID and feedforward controllers and provides calculations from
//     // them, but cannot actually set its own motor output, as accurate feedforward calculations
//     // require information from the pivot subsytem.
//     private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
//         TelescopeConstants.kS,
//         TelescopeConstants.kV);
//     private final TrapezoidProfile.Constraints constraints = 
//         new TrapezoidProfile.Constraints(3, 3);
//     private final PIDController controller = 
//         new PIDController(TelescopeConstants.kP, 0, 0);
        
//     // Stores the most recent setpoint to allow the Hold command to hold it in place
//     private TrapezoidProfile.State currentSetpoint = new TrapezoidProfile.State(0.06, 0);

//     public TelescopeSubsystem() {
//         // motor.setInverted(InvertType.None);
//         motor.setNeutralMode(NeutralModeValue.Brake);
        
//         // motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
//         // motor.setSensorPhase(false);

//         motor.configNeutralDeadband(0.004);

//         motor.configStatorCurrentLimit(
//             new StatorCurrentLimitConfiguration(true, 40, 50, 0.5));

//         // SmartDashboard.putNumber("Telescope test setpoint", 0);
//     }

//     public double getPositionM() {
//         // 4096 units per rotation, multiply rotations by diameter
//         if (Robot.isReal()) {
//             return motor.getSelectedSensorPosition() / 4096
//                 * 2 * Math.PI  * TelescopeConstants.conversionM;
//         }

//         return simPos;
//     }

//     public double getVel() {
//         return motor.getSelectedSensorVelocity() / 4096
//             * 2 * Math.PI * TelescopeConstants.conversionM * 10;
//     }

//     public double getAmps() {
//         return Math.abs(motor.getStatorCurrent().getValueAsDouble());
//     }

//      /**
//      * @return Most recent setpoint set by a move command. This setpoint only exists for utility
//      * purposes and is not used by the subsystem
//      */
//     public TrapezoidProfile.State getDesiredSetpoint() {
//         return currentSetpoint;
//     }

//     public TrapezoidProfile.Constraints getConstraints() {
//         return constraints;
//     }

//     public void toggleKill() {
//         motor.set(0);
//         dead = !dead;
//     }

//     /**
//      * Does control calculations from its ArmFeedforward and PID controllers.
//      * Does not command any motors.
//      * @param setpoint The given setpoint
//      * @param pivotAngleRad The position of the pivot, required for accurate feedforward
//      * @return The result of the calculation. Add kG * sine of the pivot angle
//      */
//     public double calculateControl(TrapezoidProfile.State setpoint, double pivotAngleRad) {
//         // SmartDashboard.putNumber("Telescope calculated", controller.calculate(getPositionM(), setpoint.position));
//         return controller.calculate(getPositionM(), setpoint.position)
//             + feedforward.calculate(setpoint.velocity)
//             // Compensates for weight of telescope as the pivot goes up
//             // Gravity Constant * Sine of Angle
//             + TelescopeConstants.kG * Math.sin(pivotAngleRad);
//     }

//     /**
//      * Resets the PID controller stored in this subsystem.
//      */
//     public void resetPID() {
//         controller.reset();
//     }

//     /**
//      * @param state The setpoint state the telescope should be driven to.
//      * Has no effect on the function of this subsytem.
//      */
//     public void setDesiredSetpoint(TrapezoidProfile.State state) {
//         currentSetpoint = state;
//     }

//     public void jogSetpointForward() {
//         currentSetpoint.position = 
//             MathUtil.clamp(
//                     currentSetpoint.position + 0.05,
//                     TelescopeConstants.minPosM,
//                     TelescopeConstants.maxPosM);
//     }

//     public void jogSetpointBackward() {
//         currentSetpoint.position = 
//             MathUtil.clamp(
//                     currentSetpoint.position - 0.05,
//                     TelescopeConstants.minPosM,
//                     TelescopeConstants.maxPosM);
//     }

//     public void setSimPos(double pos) {
//         simPos = pos;
//     }

//     public void setVolts(double input) {
//         if (!dead)
//             motor.set(ControlMode.PercentOutput, input / 12);
//     }

//     public void overrideVolts(double input) {
//         motor.set(input / 12);
//     }

//     public void stop() {
//         motor.set(0);
//     }

//     public void setBrakeMode(boolean braked) {
//         motor.setNeutralMode(braked ? NeutralModeValue.Brake : NeutralModeValue.Coast);
//     }

//     public void resetOffset() {
//         motor.setSelectedSensorPosition(0);
//     }

//     public void setP(double p) {
//         controller.setP(p);
//         controller.reset();
//     }

//     @Override
//     public void periodic() {
//         SmartDashboard.putNumber("Telescope/position", getPositionM()); 
//         SmartDashboard.putNumber("Telescope/velocity", getVel());

//         SmartDashboard.putBoolean("Telescope/dead", dead);
//         // SmartDashboard.putNumber("Telescope/applied voltage", motor.getMotorOutputVoltage());
//         SmartDashboard.putNumber("Telescope/motor current", getAmps());

//         RobotState.getInstance().putTelescopeDisplay(getPositionM());
//     }
// }