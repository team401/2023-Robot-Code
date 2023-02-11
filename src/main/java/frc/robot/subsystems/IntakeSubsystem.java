package frc.robot.subsystems;

import com.fasterxml.jackson.databind.ser.std.CalendarSerializer;
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
        leftMotor.setInverted(true);

        rightMotor.follow(leftMotor);

        leftMotor.setSmartCurrentLimit(30);
        rightMotor.setSmartCurrentLimit(30);
        
    }

    public void runForward() {
        rightMotor.set(0.45);
    }

    public void runBackward() {
        rightMotor.set(-0.45);
    }

    public void stopMotor() {
        rightMotor.stopMotor();
    }

    @Override
    public void periodic() {

    }

    
}
