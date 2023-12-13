package frc.robot.subsystems.arm.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.Constants.PivotConstants;

public class PivotIOSim implements PivotIO {

    // TODO: simulate pink-arm dynamics more accurately
    private SingleJointedArmSim armSim = new SingleJointedArmSim(
        DCMotor.getFalcon500(2),
        PivotConstants.armToMotorGearRatio,
        // TODO: weigh the arm, investigate calculating MOI manually
        SingleJointedArmSim.estimateMOI(PivotConstants.lengthWOTeleM, 0.5),
        PivotConstants.lengthWOTeleM,
        PivotConstants.maxFwdRotationRad,
        PivotConstants.maxBackRotationRad,
        true,
        0.1
    );
    private double appliedVolts = 0.0;

    public PivotIOSim() {

    }

    @Override
    public void updateInputs(PivotIOInputsAutoLogged inputs) {
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
        } else {
            appliedVolts = 0;
        }
    }
}
