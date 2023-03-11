package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.pivot.MovePivot;

public class ArmPositionCalc {
    private static final double lengthArm = 0.747; // The length of the stowed arm minus the wrist
    private static final double lengthWrist = 0.235; // The length of the wrist
    private static final double maxReach = 0.970; // The maximum reach of the telescope
    private static final double maxRotation = Math.PI/2; // The maximum rotation for the wrist
    
    public ArmPositionCalc() {}

    /**
     * Calculates the position of each of the arm segments required to reach the position of the given setpoint.
     * 
     * @param x the x coordinate (distance away) of the point
     * @param y the y coordinate of the point
     * @param theta the rotation of the wrist in radians
     * 
     * @return The rotation/position (radians/meters) of each of the arm segments in the form of an array of doubles
     * The array goes as follows: [Pivot, Telescope, Wrist]
     */
    public static double[] findPositions(double x, double y, double theta) {
        double[] result = new double[3];

        // Calculates the wrist offset
        x -= lengthWrist*Math.cos(theta);
        y -= lengthWrist*Math.sin(theta);

        // Calculates the pivot rotation
        result[0] = Math.atan2(y, x);

        // Calculates the telescope position
        result[1] = Math.sqrt(x*x + y*y) - lengthArm;

        // Spits back the wrist position
        result[2] = theta;
        System.out.print(theta);

        // Validates all positions
        if(theta > Math.abs(Math.PI/2))
            result[2] = Math.PI/2;
        if(result[1] > maxReach)
            result[1] = maxReach;
        if(result[1] < 0)
            result[1] = 0;

        return result;
    }
}
