package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
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

    private final Timer intakeTimer = new Timer();
    
    public IntakeSubsystem() {
        
        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();
        
        leftMotor.setSmartCurrentLimit(40);
        rightMotor.setSmartCurrentLimit(40);

        leftMotor.setInverted(true);
        rightMotor.setInverted(true);
        
        leftMotor.burnFlash();
        rightMotor.burnFlash();

        intakeTimer.reset();
        intakeTimer.start();

    }

    public void toggleIntake() {
        intakeMode = intakeMode == IntakeMode.Intake ? IntakeMode.None : IntakeMode.Intake;
        exceededCurrentDraw = false;
        RobotState.getInstance().setIntaked(false);
        RobotState.getInstance().setIntaking(intakeMode == IntakeMode.Intake);
        intakeTimer.reset();
        intakeTimer.start();

        if (intakeMode == IntakeMode.Intake) {
            if (RobotState.getInstance().getMode() == GamePieceMode.Cube) {
                leftMotor.set(-1);
                rightMotor.set(-1);
            }
            else  {
                leftMotor.set(1);
                rightMotor.set(1);
            }
        }
        else {
            leftMotor.stopMotor();
            rightMotor.stopMotor();
        }
    }

    public void setIntake(boolean intake) {
        intakeMode = intake ? IntakeMode.Intake : IntakeMode.None;
        exceededCurrentDraw = false;
        RobotState.getInstance().setIntaked(false);
        RobotState.getInstance().setIntaking(intakeMode == IntakeMode.Intake);
        intakeTimer.reset();
        intakeTimer.start();

        if (intakeMode == IntakeMode.Intake) {
            if (RobotState.getInstance().getMode() == GamePieceMode.Cube) {
                leftMotor.set(-1);
                rightMotor.set(-1);
            }
            else  {
                leftMotor.set(1);
                rightMotor.set(1);
            }
        }
        else {
            leftMotor.stopMotor();
            rightMotor.stopMotor();
        }
    }

    public void place() {
        intakeMode = IntakeMode.Place;
        RobotState.getInstance().setIntaked(false);
        RobotState.getInstance().setIntaking(false);

        if (RobotState.getInstance().getMode() == GamePieceMode.Cube) {
            boolean back = RobotState.getInstance().atBack();
            leftMotor.set(back ? -0.75 : 0.75);
            rightMotor.set(back ? 0.75 : -0.75);
        }
        else {
            leftMotor.set(-0.6);
            rightMotor.set(-0.6);
        }
    }

    public void shoot() {
        intakeMode = IntakeMode.Place;
        RobotState.getInstance().setIntaked(false);
        RobotState.getInstance().setIntaking(false);

        if (RobotState.getInstance().getMode() == GamePieceMode.Cube) {
            boolean back = RobotState.getInstance().atBack();
            leftMotor.set(back ? -1 : 1);
            rightMotor.set(back ? 1 : -1);
        }
        else {
            leftMotor.set(-0.8);
            rightMotor.set(-0.8);
        }
    }

    public void slowPlace() {
        intakeMode = IntakeMode.Place;
        RobotState.getInstance().setIntaked(false);
        RobotState.getInstance().setIntaking(false);

        if (RobotState.getInstance().getMode() == GamePieceMode.Cube) {
            boolean back = RobotState.getInstance().atBack();
            leftMotor.set(back ? -0.75 : 0.75);
            rightMotor.set(back ? 0.75 : -0.75);
        }
        else {
            leftMotor.set(-0.1);
            rightMotor.set(-0.1);
        }
    }

    public void stop() {
        intakeMode = IntakeMode.None;
        RobotState.getInstance().setIntaked(false);
        RobotState.getInstance().setIntaking(false);
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }

    @Override
    public void periodic() {

        double currentDraw = Math.max(leftMotor.getOutputCurrent(), rightMotor.getOutputCurrent());
        // SmartDashboard.putNumber("IntakeCurrentDraw", currentDraw);
        if (intakeMode == IntakeMode.Intake && !exceededCurrentDraw) {
            if (currentDraw > 20 && intakeTimer.hasElapsed(1)) {
                exceededCurrentDraw = true;
                RobotState.getInstance().setIntaked(true);
                if (RobotState.getInstance().getMode() == GamePieceMode.Cube) {
                    leftMotor.set(-0.35);
                    rightMotor.set(-0.35);
                }
                else {
                    leftMotor.set(0.35);
                    rightMotor.set(0.35);
                }
            }
        }

    }
    
}