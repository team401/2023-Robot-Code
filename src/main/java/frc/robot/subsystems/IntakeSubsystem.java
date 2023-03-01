package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.GamePieceMode;

public class IntakeSubsystem extends SubsystemBase {
    private static enum IntakeMode {
        None, Intake, Place
    }
    private CANSparkMax leftMotor = new CANSparkMax(
        CANDevices.leftIntakeMotorID, MotorType.kBrushed);
    private CANSparkMax rightMotor = new CANSparkMax(
        CANDevices.rightIntakeMotorID, MotorType.kBrushed);

    private IntakeMode intakeMode = IntakeMode.None;
    private boolean exceededCurrentDraw = false;
    
    public IntakeSubsystem() {
        
        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();
        
        leftMotor.setSmartCurrentLimit(40);
        rightMotor.setSmartCurrentLimit(40);
        
        leftMotor.burnFlash();
        rightMotor.burnFlash();

    }

    public void toggleIntake() {
        intakeMode = intakeMode == IntakeMode.Intake ? IntakeMode.None : IntakeMode.Intake;
        exceededCurrentDraw = false;
        RobotState.getInstance().setIntaked(false);
    }

    public void place() {
        intakeMode = IntakeMode.Place;
        RobotState.getInstance().setIntaked(false);
    }

    public void stop() {
        intakeMode = IntakeMode.None;
        RobotState.getInstance().setIntaked(false);
    }

    @Override
    public void periodic() {

        if (intakeMode == IntakeMode.Intake) {
            double currentDraw = (leftMotor.getOutputCurrent() + rightMotor.getOutputCurrent()) / 2;
            SmartDashboard.putNumber("IntakeCurrentDraw", currentDraw);
            if (!exceededCurrentDraw) {
                if (RobotState.getInstance().getMode() == GamePieceMode.Cube) {
                    leftMotor.set(-0.75);
                    rightMotor.set(-0.75);
                }
                else if (RobotState.getInstance().getMode() == GamePieceMode.ConeBack) {
                    leftMotor.set(0.75);
                    rightMotor.set(0.75);
                }
                if ((RobotState.getInstance().getMode() == GamePieceMode.Cube && currentDraw > 50) || 
                    (RobotState.getInstance().getMode() == GamePieceMode.Cube && currentDraw > 50)) {
                    exceededCurrentDraw = true;
                    RobotState.getInstance().setIntaked(true);
                }
                
            }
            else {
                if (RobotState.getInstance().getMode() == GamePieceMode.Cube) {
                    leftMotor.set(-0.25);
                    rightMotor.set(-0.25);
                }
                else if (RobotState.getInstance().getMode() == GamePieceMode.ConeBack) {
                    leftMotor.set(0.25);
                    rightMotor.set(0.25);
                }
            }
        }
        else if (intakeMode == IntakeMode.Place) {
            if (RobotState.getInstance().getMode() == GamePieceMode.Cube) {
                boolean back = RobotState.getInstance().atBack();
                leftMotor.set(back ? -0.75 : 0.75);
                rightMotor.set(back ? 0.75 : -0.75);
            }
            if (RobotState.getInstance().getMode() == GamePieceMode.ConeBack) {
                leftMotor.set(-0.75);
                rightMotor.set(-0.75);
            }
        }
        else {
            leftMotor.stopMotor();
            rightMotor.stopMotor();
        }

    }

    
}
