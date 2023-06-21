package frc.robot.subsystems.telescope;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants.TelescopeConstants;

public class TelescopeIOSim implements TelescopeIO {
    private ElevatorSim sim = new ElevatorSim(DCMotor.getFalcon500(1),
                                            TelescopeConstants.conversionM,
                                            TelescopeConstants.weightKg,
                                            0.05,
                                            TelescopeConstants.minPosMeters,
                                            TelescopeConstants.maxPosMeters,
                                            false);

    double appliedVolts = 0.0;

    public TelescopeIOSim() {}

    @Override
    public void updateInputs(TelescopeIOInputs inputs) {
        sim.setInputVoltage(appliedVolts);

        sim.update(0.02);

        inputs.appliedVolts = appliedVolts;
        inputs.positionMeters = sim.getPositionMeters();
        inputs.velocityMetersPerSec = sim.getVelocityMetersPerSecond();
        inputs.currentAmps = sim.getCurrentDrawAmps();
    }

    @Override
    public void setVolts(double volts) {
        appliedVolts = volts;
    }
}
