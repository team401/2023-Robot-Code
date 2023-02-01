package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.PivotSubsystem;

public class CharacterizePivot {
    public class FindKS extends CommandBase {
        private PivotSubsystem pivot;

        private double volts;

        public FindKS(PivotSubsystem pivot) {
            this.pivot = pivot;
        }

        @Override
        public void execute() {
            pivot.setVolts(volts);
            volts += 0.01;

            SmartDashboard.putNumber("Pivot kS Volts", volts);
        }

        @Override
        public void end(boolean interrupted) {
            pivot.setVolts(0);
        }
    }

    public class FindKV extends CommandBase {
        private PivotSubsystem pivot;

        public FindKV(PivotSubsystem pivot) {
            this.pivot = pivot;
        }

        @Override
        public void initialize() {
            pivot.setVolts(1);
        }

        @Override
        public boolean isFinished() {
            return pivot.getPositionRad() == Math.PI / 2;
        }

        @Override
        public void end(boolean isFinished) {
            pivot.setVolts(0);
            SmartDashboard.putNumber("Pivot Speed at Top Rad/s", pivot.getVelRadS());
        }
    }

    public class FindKG extends CommandBase{
        private PivotSubsystem pivot;

        public FindKG(PivotSubsystem pivot) {
            this.pivot = pivot;
        }

        @Override
        public void initialize() {
            pivot.setVolts(1);
        }

        @Override
        public boolean isFinished() {
            return pivot.getPositionRad() == 0;
        }

        @Override
        public void end(boolean isFinished) {
            pivot.setVolts(0);
            SmartDashboard.putNumber("Pivot Speed at Level Rad/s", pivot.getVelRadS());
        }
    }
}