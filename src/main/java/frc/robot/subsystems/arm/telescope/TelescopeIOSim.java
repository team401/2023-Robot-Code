package frc.robot.subsystems.arm.telescope;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;
import frc.robot.Constants.TelescopeConstants;

public class TelescopeIOSim implements TelescopeIO {

    // TODO: simulate pink-arm dynamics more accurately
    private ElevatorSim elevatorSim = new ElevatorSim(
            LinearSystemId.createElevatorSystem(
                    DCMotor.getFalcon500(1),
                    // TODO: find actual mass
                    3,
                    // TODO: find actual radius
                    0.03,
                    // TODO: find actual gear ratio
                    0.125),
            DCMotor.getFalcon500(1),
            0.0,
            TelescopeConstants.maxPosM,
            true,
            0.1);
    private double appliedVolts = 0.0;

    public TelescopeIOSim() {

    }

    @Override
    public void updateInputs(TelescopeIOInputsAutoLogged inputs) {
        elevatorSim.setInputVoltage(appliedVolts);
        elevatorSim.update(Constants.loopTime);

        inputs.positionM = elevatorSim.getPositionMeters();
        inputs.velocityMS = elevatorSim.getVelocityMetersPerSecond();
        inputs.appliedVolts = this.appliedVolts;
        inputs.statorCurrent = elevatorSim.getCurrentDrawAmps();
    }

    @Override
    public void setOutput(double volts) {
        if (DriverStation.isEnabled()) {
            appliedVolts = MathUtil.clamp(volts, -12, 12);
        }
    }
}
