package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.DriveConstants;

public class ModuleIOSim implements ModuleIO {
    // jKg constants unknown, stolen from Mechanical Advnatage
    private FlywheelSim driveSim =
        new FlywheelSim(DCMotor.getFalcon500(1), DriveConstants.driveWheelGearReduction, 0.025);
    private FlywheelSim rotationSim =
        new FlywheelSim(DCMotor.getFalcon500(1), DriveConstants.rotationWheelGearReduction, 0.004096955);

    private double drivePositionRad = 0.0;
    private double rotationPositionRad = 0.0;
    private double driveAppliedVolts = 0.0;
    private double rotationAppliedVolts = 0.0;

    private Timer dtTimer = new Timer();

    public ModuleIOSim() {
        dtTimer.reset();
        dtTimer.start();
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        double loopTime = dtTimer.get();

        driveSim.update(loopTime);
        rotationSim.update(loopTime);

        double angleDiffRad =
            rotationSim.getAngularVelocityRadPerSec() * loopTime;
        rotationPositionRad += angleDiffRad;

        drivePositionRad += driveSim.getAngularVelocityRadPerSec() * loopTime;

        inputs.drivePositionRad = drivePositionRad;
        inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.driveCurrentAmps = driveSim.getCurrentDrawAmps();

        inputs.rotationPositionRad = MathUtil.angleModulus(rotationPositionRad);
        inputs.rotationVelocityRadPerSec = rotationSim.getAngularVelocityRadPerSec();
        inputs.rotationAppliedVolts = rotationAppliedVolts;
        inputs.rotationCurrentAmps = rotationSim.getCurrentDrawAmps();
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveSim.setInputVoltage(volts);
        driveAppliedVolts = volts;
    }

    @Override
    public void setRotationVoltage(double volts) {
        rotationSim.setInputVoltage(volts);
        rotationAppliedVolts = volts;
    }

    @Override
    public void zeroEncoders() {
        rotationPositionRad = 0.0;
        drivePositionRad = 0.0;
    }
    // we don't need to implement togglekill()
}
