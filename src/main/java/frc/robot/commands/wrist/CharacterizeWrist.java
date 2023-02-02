package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class CharacterizeWrist {
    public class FindKS extends CommandBase {
        private WristSubsystem wrist;

        private double volts;

        public FindKS(WristSubsystem w) {
            wrist = w;
        }

        @Override
        public void execute() {
            wrist.setVolts(volts);
            volts += 0.01;

            SmartDashboard.putNumber("Wrist kS Volts", volts);
        }

        @Override
        public void end(boolean interrupted) {
            wrist.setVolts(0);
        }
    }

    public class FindKV extends CommandBase {
        private WristSubsystem wrist;
        private PivotSubsystem pivot;

        public FindKV(WristSubsystem w, PivotSubsystem p) {
            wrist = w;
            pivot = p;
        }

        @Override
        public void initialize() {
            wrist.setVolts(1);
        }

        @Override
        public boolean isFinished() {
            return wrist.getPositionRad() + pivot.getPositionRad() == Math.PI / 2;
        }

        @Override
        public void end(boolean isFinished) {
            wrist.setVolts(0);
            SmartDashboard.putNumber("Wrist Speed at Top Rad/s", wrist.getVelRadS());
        }
    }

    public class FindKG extends CommandBase{
        private WristSubsystem wrist;
        private PivotSubsystem pivot;

        public FindKG(WristSubsystem w, PivotSubsystem p) {
            wrist = w;
            pivot = p;
        }

        @Override
        public void initialize() {
            pivot.setVolts(1);
        }

        @Override
        public boolean isFinished() {
            return wrist.getPositionRad() + pivot.getPositionRad() == 0;
        }

        @Override
        public void end(boolean isFinished) {
            wrist.setVolts(0);
            SmartDashboard.putNumber("Wrist Speed at Level Rad/s", wrist.getVelRadS());
        }
    }
}
