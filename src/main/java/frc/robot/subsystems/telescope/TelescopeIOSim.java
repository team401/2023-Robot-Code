package frc.robot.subsystems.telescope;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import frc.robot.Constants.TelescopeConstants;

public class TelescopeIOSim implements TelescopeIO {
    private LinearSystemSim sim = new LinearSystemSim(
        DCMotor.getFalcon500(1),
        TelescopeConstants.conversionM,
        TelescopeConstants.kG,
        TelescopeConstants.weightKg,
        LinearSystem.estimateMOI(TelescopeConstants.conversionM, TelescopeConstants.weightKg),
        TelescopeConstants.kMinHeightM,
        TelescopeConstants.kMaxHeightM,
        true
    );

    double appliedVolts = 0.0;

    public TelescopeIOSim() {}

    @Override
    public void updateInputs(TelescopeIOInputs inputs) {
        sim.setInputVoltage(appliedVolts);

        sim.update(0.02);

        inputs.appliedVolts = appliedVolts;
        inputs.positionMeters = sim.getPositionMeters();
        inputs.velocityMetersPerSec = sim.getVelocityRadPerSec();
        inputs.currentAmps = sim.getCurrentDrawAmps();
    }

    @Override
    public void setVolts(double volts) {
        appliedVolts = volts;
    }
}
