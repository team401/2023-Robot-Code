package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.PivotSubsystem;

/**
 * Set of commands meant to characterize the pivot
 */
public class CharacterizePivot {
    /**
     * Command to find the kS constant of the pivot. <p>
     * 
     * Must be done visually. Bind this command to a button and hold the 
     * button until the pivot visually moves. Release the button immediatly 
     * when it begins moving, and the estimated kS will be printed on 
     * SmartDashboard.
     */
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

    /**
     * Command to find the kV constant of the pivot. <p>
     * 
     * Finds the velocity of the pivot when given 1 volt exactly when it passes
     * the 90-degree point, in order to ignore the effects of gravity. Prints
     * the speed to SmartDashboard when the command ends.
     */
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

    /**
     * Finds the kG constant of the pivot. <p>
     * 
     * Very simmilar to finding the kV, but makes the speed measurement at 
     * 0 degrees: level with the ground.
     */
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