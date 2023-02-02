package frc.robot.commands.telescope;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TelescopeConstants;
import frc.robot.subsystems.TelescopeSubsystem;

public class CharacterizeTelescope {

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
