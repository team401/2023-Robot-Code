package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.PIDCommand;
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

    private PIDController driveController = new PIDController(0.9, 0, 0);
    private double driveSetpointRadPerS = 0;
    private double driveFFVolts = 0;

    // I don't trust the loop time
    private Timer dtTimer = new Timer();

    public ModuleIOSim() {
        dtTimer.reset();
        dtTimer.start();
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        double loopTime = dtTimer.get();
        dtTimer.reset();

        driveSim.update(loopTime);
        rotationSim.update(loopTime);

        // `FlywheelSim` does not specify position, so we have to calculate it manually
        rotationPositionRad += rotationSim.getAngularVelocityRadPerSec() * loopTime;
        drivePositionRad += driveSim.getAngularVelocityRadPerSec() * loopTime;

        inputs.drivePositionRad = drivePositionRad;
        inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.driveCurrentAmps = driveSim.getCurrentDrawAmps();

        inputs.rotationPositionRad = MathUtil.angleModulus(rotationPositionRad);
        inputs.rotationVelocityRadPerSec = rotationSim.getAngularVelocityRadPerSec();
        inputs.rotationAppliedVolts = rotationAppliedVolts;
        inputs.rotationCurrentAmps = rotationSim.getCurrentDrawAmps();

        inputs.driveSetpointRadPerS = driveSetpointRadPerS;

        setDriveVoltage(driveController.calculate(
            driveSim.getAngularVelocityRadPerSec(), driveSetpointRadPerS) + driveFFVolts);
        // setDriveVoltage((driveSetpointRadPerS - driveSim.getAngularVelocityRadPerSec()) * 0.5 + driveFFVolts);
    }

    @Override
    public void setDriveVoltage(double volts) {
        volts = MathUtil.clamp(volts, -12, 12);
        driveSim.setInputVoltage(volts);
        driveAppliedVolts = volts;
    }

    @Override
    public void setRotationVoltage(double volts) {
        volts = MathUtil.clamp(volts, -12, 12);
        rotationSim.setInputVoltage(volts);
        rotationAppliedVolts = volts;
    }

    @Override
    public void zeroEncoders() {
        rotationPositionRad = 0.0;
        drivePositionRad = 0.0;
    }

    @Override
    public void setDriveVelocity(double velocityRadPerS, double ffVolts) {
        driveSetpointRadPerS = velocityRadPerS;
        driveFFVolts = ffVolts;
    }

    @Override
    public void setDrivePD(double p, double d) {
        driveController.setP(p);
        driveController.setD(d);
    }

    // we don't need to implement togglekill()
}
