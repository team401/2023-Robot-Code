package frc.robot.commands.FeedForward;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class TuneArmG extends CommandBase {
    private ArmSubsystem arm;

    double startPosition;

    double kG;
    double kS;

    public TuneArmG(ArmSubsystem arm, double kS) {
        this.arm = arm;
        this.kS = kS;
    }

    @Override
    public void initialize() {
        startPosition = arm.getPositionRad();
        kG = kS;
    }

    @Override
    public void execute() {
        arm.setVolts(-kG);
        kG += 0.001;
    }

    @Override
    public void end(boolean interrupted) {
        arm.stop();
        SmartDashboard.putNumber("kG", kG - kS);
    }

    @Override
    public boolean isFinished() {
        return arm.getPositionRad() > Math.abs(startPosition - 0.1);
    }
}
