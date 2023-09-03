package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** 
 * CommandJoystick child class that wraps around our Thrustmaster joysticks.
 * Use instead of `new JoystickButton()`
 */
public class Thrustmaster extends CommandJoystick {
    
    public Thrustmaster(int id) {
        super(id);
        // TODO: set x, y, twist, and slider channels
    }

    public Trigger topCenter() {
        return super.button(2);
    }

    public Trigger topLeft() {
        return super.button(3);
    }

    public Trigger topRight() {
        return super.button(4);
    }

    /**
     * For use with bottom 12 buttons
     * 
     * <p>
     * WARNING: The button ids change depending on if the controller is in
     * 'left' or 'right' mode. Check the switch on the bottom.
     * <p>
     * RIGHT:<p> 
     *  5 6 7 | 13 12 11<p>
     * 10 9 8 | 14 15 16<p>
     * 
     * LEFT:<p>
     *  11 12 13 | 7 6 5<p>
     *  16 15 14 | 8 9 10
     */
    public Trigger lowerButton(int button) {
        return super.button(button);
    }
}
