package frc.robot.subsystems.pivot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.PivotConstants;

public class PivotIOSim implements PivotIO {
    private SingleJointedArmSim sim = new SingleJointedArmSim(DCMotor.getFalcon500(1),
                                                            PivotConstants.armToMotorGearRatio,
                                                            SingleJointedArmSim.estimateMOI(PivotConstants.lengthWOTeleM, PivotConstants.singleextraKg),
                                                            PivotConstants.lengthWOTeleM,
                                                            PivotConstants.maxFwdRotationRad,
                                                            PivotConstants.maxBackRotationRad,
                                                            true);
    double appliedVolts = 0.0;

    public PivotIOSim() {}

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        sim.setInputVoltage(appliedVolts);

        sim.update(0.02);

        inputs.appliedVoltsSim = appliedVolts;
        inputs.positionRad = sim.getAngleRads();
        inputs.velocityRadPerSec = sim.getVelocityRadPerSec();
        inputs.currentAmpsSim = sim.getCurrentDrawAmps();
    }

    @Override
    public void setVolts(double volts) {
        this.appliedVolts = volts;
    }
}
