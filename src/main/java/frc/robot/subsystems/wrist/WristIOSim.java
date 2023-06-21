package frc.robot.subsystems.wrist;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.WristConstants;

public class WristIOSim implements WristIO {
    private SingleJointedArmSim sim = new SingleJointedArmSim(DCMotor.getFalcon500(1),
                                                            WristConstants.gearRatio,
                                                            SingleJointedArmSim.estimateMOI(WristConstants.intakeLengthM, WristConstants.intakeWeightKg),
                                                            WristConstants.intakeLengthM,
                                                            WristConstants.negativeLimitRad,
                                                            WristConstants.positiveLimitRad,
                                                            true);
    double appliedVolts = 0.0;

    public WristIOSim() {}

    @Override
    public void updateInputs(WristIOInputs inputs) {
        sim.setInputVoltage(appliedVolts);

        sim.update(0.02);

        inputs.appliedVolts = appliedVolts;
        inputs.positionRad = sim.getAngleRads();
        inputs.velocityRadPerSec = sim.getVelocityRadPerSec();
        inputs.currentAmps = sim.getCurrentDrawAmps();
    }

    @Override
    public void setVolts(double volts) {
        this.appliedVolts = volts;
    }
}
