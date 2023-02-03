package frc.robot.commands.telescope;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TelescopeSubsystem;

/**
 * Set of commands meant to characterize the pivot
 */
public class CharacterizeTelescope {
    /**
     * Command to find the kS constant of the telescope. <p>
     * 
     * Must be done visually. Bind this command to a button and hold the 
     * button until the telescope visually moves. Release the button immediatly
     * when it begins moving, and the estimated kS will be printed on 
     * SmartDashboard.
     */
    public class FindKS extends CommandBase {
        private TelescopeSubsystem telescope;
        private double volts;
        
        public FindKS(TelescopeSubsystem t) {
            telescope = t;
        }

        @Override
        public void execute() {
            telescope.setVolts(volts);
            volts += 0.01;
            SmartDashboard.putNumber("Telescope kS Volts", volts);
        }

        @Override
        public void end(boolean interrupted) {
            telescope.setVolts(0);
        }
    }

    /**
     * Command to find the kV and kG of the telescope.  <p>
     * 
     * For kV: Position the pivot horizontally, and push the button for a few
     * seconds. <p>
     * 
     * For kG: Position the pivot vertically, and do the same. <p>
     * 
     * The command needs to be manually toggled by holding a button, and will
     * print out the velocity of the telescope when given 1 volt.
     */
    public class FindKV extends CommandBase {
        private TelescopeSubsystem telescope;

        public FindKV(TelescopeSubsystem t) {
            telescope = t;
        }

        @Override
        public void initialize() {
            telescope.setVolts(1);
        }

        @Override
        public void execute() {
            SmartDashboard.putNumber("Telescope Speed", telescope.getVel());
        }

        @Override
        public void end(boolean interrupted) {
            telescope.setVolts(0);
        }
    }
    
}
