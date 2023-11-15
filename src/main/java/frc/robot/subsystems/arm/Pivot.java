package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.CANDevices;

public class Pivot extends GenericArmJoint {

    private TalonFX rightMotor = new TalonFX(CANDevices.rightPivotMotorID, CANDevices.canivoreName);
    private DutyCycleEncoder encoder = new DutyCycleEncoder(CANDevices.pivotEncoderID);
    private TalonFX leftMotor = new TalonFX(CANDevices.leftPivotMotorID, CANDevices.canivoreName);

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

    
    public Pivot() {

    }
}
