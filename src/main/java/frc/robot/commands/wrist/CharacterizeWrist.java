package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.WristSubsystem;

/**
 * Set of commands meant to characterize the wrist
 */
public class CharacterizeWrist {
    /**
     * Command to find the kS constant of the wrist. <p>
     * 
     * Must be done visually. Bind this command to a button and hold the 
     * button until the wrist visually moves. Release the button immediatly 
     * when it begins moving, and the estimated kS will be printed on 
     * SmartDashboard.
     */
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

    /**
     * Command to find the kV constant of the wrist. <p>
     * 
     * Finds the velocity of the wrist when given 1 volt exactly when it passes
     * the 90-degree point, in order to ignore the effects of gravity. Prints
     * the speed to SmartDashboard when the command ends.
     */
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

    /**
     * Finds the kG constant of the wrist. <p>
     * 
     * Very simmilar to finding the kV, but makes the speed measurement at 
     * 0 degrees: level with the ground.
     */
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
