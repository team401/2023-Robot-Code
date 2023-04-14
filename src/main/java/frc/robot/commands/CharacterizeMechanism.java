package frc.robot.commands;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
Set as default command of subsystem to be characterized
Use Y/A buttons to change value by 0.01
Use X/B buttons to change value by 0.1
Use right bumper to run value

To help characterize the telescope, you could use:
telescope.setDefaultCommand(new CharacterizeMechanism(telescope, masher.getGamepad(), (v) -> telescope.overrideVolts(v), 0.3));

If you wanted to tune the proportional of the telescope controller:
telescope.setDefaultCommand(new CharacterizeMechanism(telescope, masher.getGamepad(), (p) -> telescope.setP(p), 5));
*/

public class CharacterizeMechanism extends CommandBase {

    private final XboxController controller;
    private final Consumer<Double> setValue;

    private double value;

    public CharacterizeMechanism(SubsystemBase subsystem, XboxController controller, Consumer<Double> setValue, double initialValue) {

        this.controller = controller;
        this.setValue = setValue;

        value = initialValue;

        addRequirements(subsystem);

    }

    @Override
    public void execute() {
        
        if (controller.getRightBumperPressed()) {
            setValue.accept(value);
        }
        else if (controller.getRightBumperReleased()) {
            setValue.accept(0.0);
        }

        if (controller.getLeftBumperPressed()) {
            setValue.accept(-value);
        }
        else if (controller.getLeftBumperReleased()) {
            setValue.accept(0.0);
        }

        if (controller.getYButtonPressed()) {
            value += 0.01;
        }
        if (controller.getAButtonPressed()) {
            value -= 0.01;
        }
        
        if (controller.getXButtonPressed()) {
            value += 0.1;
        }
        if (controller.getBButtonPressed()) {
            value -= 0.1;
        }
        
        SmartDashboard.putNumber("CharacterizationValue", value);

    }
    
}
