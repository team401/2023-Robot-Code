package frc.robot.subsystems.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.WristConstants;

public class WristIOSim implements WristIO {
    private SingleJointedArmSim sim = new SingleJointedArmSim(DCMotor.getFalcon500(1), WristConstants.gearRatio, SingleJointedArmSim.estimateMOI(WristConstants.intakeLengthM, WristConstants.intakeWeightKg), WristConstants.intakeLengthM, WristConstants.negativeLimitRad, WristConstants.positiveLimitRad, true);
    private PIDController pid = new PIDController(0.0, 0.0, 0.0);
    double volts = 0.0;
    double appliedVolts = 0.0;

    Timer time = new Timer();

    public WristIOSim() {
        time.start();
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        appliedVolts = MathUtil.clamp(
            pid.calculate(sim.getVelocityRadPerSec()) + volts, -12.0,
            12.0);
        sim.setInputVoltage(appliedVolts);

        sim.update(0.02);

        inputs.appliedVolts = appliedVolts;
        inputs.positionRad = sim.getAngleRads();
        inputs.velocityRadPerSec = sim.getVelocityRadPerSec();
        inputs.currentAmps = sim.getCurrentDrawAmps();
    }

    @Override
    public void setVolts(double volts) {
        this.volts = volts;
    }

    @Override
    public void setPosition(double positionRad) {
        pid.setSetpoint(positionRad);
    }

    @Override
    public void configurePID(double kP, double kI, double kD) {
        pid.setPID(kP, kI, kD);
    }
}
