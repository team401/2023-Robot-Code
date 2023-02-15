package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;

public class IntakeSubsystem extends SubsystemBase {
    private CANSparkMax leftMotor = new CANSparkMax(
        CANDevices.leftIntakeMotorID, MotorType.kBrushed);
    private CANSparkMax rightMotor = new CANSparkMax(
        CANDevices.rightIntakeMotorID, MotorType.kBrushed);
    
    public IntakeSubsystem() {
        
        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();

        leftMotor.setInverted(true);
        
        leftMotor.setSmartCurrentLimit(30);
        rightMotor.setSmartCurrentLimit(30);
        
        leftMotor.burnFlash();
        rightMotor.burnFlash();

    }

    public void runForward() {
        leftMotor.set(0.45);
        rightMotor.set(0.45);
    }

    public void runBackward() {
        leftMotor.set(-0.45);
        rightMotor.set(-0.45);
    }

    public void stopMotor() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }

    @Override
    public void periodic() {

    }

    
}
