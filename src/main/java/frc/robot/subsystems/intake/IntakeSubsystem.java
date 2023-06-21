package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.Constants.GamePieceMode;

public class IntakeSubsystem extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private static enum IntakeMode {
        None, Intake, Place
    }

    private IntakeMode intakeMode = IntakeMode.None;
    private boolean exceededCurrentDraw = false;

    private final Timer intakeTimer = new Timer();
    
    public IntakeSubsystem(IntakeIO io) {
        this.io = io;

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
                io.setWheelVelocitiesVolts(-12, -12);
            }
            else  {
                io.setWheelVelocitiesVolts(12, 12);
            }
        }
        else {
            io.setWheelVelocitiesVolts(0, 0);
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
                io.setWheelVelocitiesVolts(-12, -12);
            }
            else  {
                io.setWheelVelocitiesVolts(12, 12);
            }
        }
        else {
            io.setWheelVelocitiesVolts(0, 0);
        }
    }

    public void place() {
        intakeMode = IntakeMode.Place;
        RobotState.getInstance().setIntaked(false);
        RobotState.getInstance().setIntaking(false);

        if (RobotState.getInstance().getMode() == GamePieceMode.Cube) {
            boolean back = RobotState.getInstance().atBack();
            io.setWheelVelocitiesVolts(back ? -9 : 9, back ? 9 : -9);
        }
        else {
            io.setWheelVelocitiesVolts(-7, -7);
        }
    }

    public void shoot() {
        intakeMode = IntakeMode.Place;
        RobotState.getInstance().setIntaked(false);
        RobotState.getInstance().setIntaking(false);

        if (RobotState.getInstance().getMode() == GamePieceMode.Cube) {
            boolean back = RobotState.getInstance().atBack();
            io.setWheelVelocitiesVolts(back ? -12 : 12, back ? 12 : -12);
        }
        else {
            io.setWheelVelocitiesVolts(-10, -10);
        }
    }

    public void shootBackwards() {
        intakeMode = IntakeMode.Place;
        RobotState.getInstance().setIntaked(false);
        RobotState.getInstance().setIntaking(false);

        if (RobotState.getInstance().getMode() == GamePieceMode.Cube) {
            boolean back = RobotState.getInstance().atBack();
            io.setWheelVelocitiesVolts(back ? -12 : 12, back ? 12 : -12);
        }
        else {
            io.setWheelVelocitiesVolts(-10, -10);
        }
    }

    public void slowPlace() {
        intakeMode = IntakeMode.Place;
        RobotState.getInstance().setIntaked(false);
        RobotState.getInstance().setIntaking(false);

        if (RobotState.getInstance().getMode() == GamePieceMode.Cube) {
            boolean back = RobotState.getInstance().atBack();
            io.setWheelVelocitiesVolts(back ? -9 : 9, back ? 9 : -9);
        }
        else {
            io.setWheelVelocitiesVolts(-1.2, -1.2);
        }
    }

    public void stop() {
        intakeMode = IntakeMode.None;
        RobotState.getInstance().setIntaked(false);
        RobotState.getInstance().setIntaking(false);
        io.setWheelVelocitiesVolts(0, 0);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        Logger.getInstance().processInputs("Intake", inputs);

        double currentDraw = Math.max(inputs.leftCurrentAmps, inputs.rightCurrentAmps);
        Logger.getInstance().recordOutput("Intake/TopCurrentDraw", currentDraw);
        if (intakeMode == IntakeMode.Intake && !exceededCurrentDraw) {
            if (currentDraw > 20 && intakeTimer.hasElapsed(1)) {
                exceededCurrentDraw = true;
                RobotState.getInstance().setIntaked(true);
                if (RobotState.getInstance().getMode() == GamePieceMode.Cube) {
                    io.setWheelVelocitiesVolts(-4.2, -4.2);
                }
                else {
                    io.setWheelVelocitiesVolts(4.2, 4.2);
                }
            }
        }

    }
    
}