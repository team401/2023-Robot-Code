package frc.robot.subsystems.arm.wrist;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.LinearSystem;
import frc.robot.Constants;
import frc.robot.Constants.WristConstants;

public class WristIOSim implements WristIO {
    
    private SingleJointedArmSim armSim = new SingleJointedArmSim(
        LinearSystemId.createSingleJointedArmSystem(
            DCMotor.getFalcon500(1),
            SingleJointedArmSim.estimateMOI(WristConstants.intakeLengthM, 1),
            WristConstants.gearRatio),
        DCMotor.getFalcon500(1),
        WristConstants.gearRatio,
        WristConstants.intakeLengthM,
        WristConstants.negativeLimitRad,
        WristConstants.positiveLimitRad,
        false,
        0.2);
    private double appliedVolts = 0.0;

    public WristIOSim() {

    }

    @Override
    public void updateInputs(WristIOInputsAutoLogged inputs) {
        armSim.setInputVoltage(appliedVolts);
        armSim.update(Constants.loopTime);

        inputs.positionRad = armSim.getAngleRads();
        inputs.velocityRadS = armSim.getVelocityRadPerSec();
        inputs.appliedVolts = this.appliedVolts;
        inputs.statorCurrent = armSim.getCurrentDrawAmps();
    }

    @Override
    public void setOutput(double volts) {
        if (DriverStation.isEnabled()) {
            appliedVolts = MathUtil.clamp(volts, -12, 12);
        }
    }
}
