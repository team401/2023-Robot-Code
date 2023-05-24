package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Timer;

/**
 *  Very simple proof-of-concept for AdvantageKit simulation
 */
public class IntakeIOSim implements IntakeIO {
    double leftVolts = 0.0;
    double rightVolts = 0.0;

    Timer time = new Timer();

    public IntakeIOSim() {
        time.start();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.leftOutputVolts = leftVolts; // motor.getOutputVoltage()
        inputs.rightOutputVolts = rightVolts; 

        inputs.leftCurrentAmps = 0.0;
        inputs.leftCurrentAmps = 0.0;
    }

    @Override
    public void setWheelVelocitiesVolts(double left, double right) {
        leftVolts = left;
        rightVolts = right;
    }
}
