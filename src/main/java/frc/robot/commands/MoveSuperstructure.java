package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.pivot.MovePivot;
import frc.robot.commands.telescope.MoveTelescope;
import frc.robot.commands.wrist.MoveWrist;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.util.ArmPositionCalc;

public class MoveSuperstructure extends ParallelCommandGroup {
    private static final double lengthArm = 0.747; // The length of the stowed arm minus the wrist
    private static final double lengthWrist = 0.235; // The length of the wrist
    private static final double maxReach = 0.970; // The maximum reach of the telescope

    private double armPositionX;
    private double armPositionY;
    private double armPositionTheta;

    public MoveSuperstructure(PivotSubsystem pivot, TelescopeSubsystem telescope, WristSubsystem wrist, double changeX, double changeY, double changeTheta) {
        armPositionX += changeX;
        armPositionY += changeY;
        armPositionTheta += changeTheta;

        double[] result = new double[3];

        // Calculates the wrist offset
        // x -= lengthWrist*Math.cos(theta);
        // y -= lengthWrist*Math.sin(theta);

        // // Calculates the pivot rotation
        // result[0] = Math.atan2(y, x);

        // // Calculates the telescope position
        // result[1] = Math.sqrt(x*x + y*y) - lengthArm;

        // // Spits back the wrist position
        // result[2] = theta;
        
        // // Validates all positions
        // if(theta > Math.abs(Math.PI/2))
        //     result[2] = Math.PI/2;
        // if(result[1] > maxReach)
        //     result[1] = maxReach;
        // if(result[1] < 0)
        //     result[1] = 0;

        // double[] pos = ArmPositionCalc.findPositions(changeX, changeY, changeTheta);
        // addCommands(new MovePivot(pivot, telescope, () -> pos[0]),
        //             new MoveTelescope(telescope, pivot, () -> pos[1], () -> pos[1]),
        //             new MoveWrist(wrist, pivot, () -> pos[2]));
    }
}
