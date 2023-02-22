package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;

public class Balance extends CommandBase {

    private final Drive drive;

    public Balance(Drive drive) {
        this.drive = drive;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        
    }
    
}
