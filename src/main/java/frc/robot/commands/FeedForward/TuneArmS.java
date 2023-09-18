package frc.robot.commands.FeedForward;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class TuneArmS extends CommandBase {
    private ArmSubsystem arm;

    double startPosition;

    double appliedVolts;

    public TuneArmS(ArmSubsystem arm) {
        this.arm = arm;
    }

    @Override
    public void initialize() {
        startPosition = arm.getPositionRad();
        appliedVolts = 0;
    }

    @Override
    public void execute() {
        arm.setVolts(appliedVolts);
        appliedVolts += 0.001;
    }

    @Override
    public void end(boolean interrupted) {
        arm.stop();
        SmartDashboard.putNumber("kS", appliedVolts);
    }

    @Override
    public boolean isFinished() {
        return arm.getPositionRad() > Math.abs(startPosition - 20);
    }
}
