package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.GamePieceMode;

public class IntakeSubsystem extends SubsystemBase {
    private CANSparkMax leftMotor = new CANSparkMax(
        CANDevices.leftIntakeMotorID, MotorType.kBrushed);
    private CANSparkMax rightMotor = new CANSparkMax(
        CANDevices.rightIntakeMotorID, MotorType.kBrushed);
    
    public IntakeSubsystem() {
        
        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();
        
        leftMotor.setSmartCurrentLimit(30);
        rightMotor.setSmartCurrentLimit(30);
        
        leftMotor.burnFlash();
        rightMotor.burnFlash();

    }

    public void intake() {
        if (RobotState.getInstance().getMode() == GamePieceMode.Cube) {
            leftMotor.set(-0.45);
            rightMotor.set(-0.45);
        }
        if (RobotState.getInstance().getMode() == GamePieceMode.ConeBack) {
            leftMotor.set(0.45);
            rightMotor.set(0.45);
        }
    }

    public void place() {
        if (RobotState.getInstance().getMode() == GamePieceMode.Cube) {
            boolean back = RobotState.getInstance().atBack();
            leftMotor.set(back ? -0.45 : 0.45);
            rightMotor.set(back ? 0.45 : -0.45);
        }
        if (RobotState.getInstance().getMode() == GamePieceMode.ConeBack) {
            leftMotor.set(-0.45);
            rightMotor.set(-0.45);
        }
    }

    public void stopMotor() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }

    @Override
    public void periodic() {

    }

    
}
