package frc.robot.util;

import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.WristConstants;

public class PositionConverter {
    public double[] wristEndpointToRegular(double[] positions) {
        // Endpoint of the arm ignoring the wrist
        double[] armPoint = 
            {positions[0] + WristConstants.intakeLengthM 
                * Math.cos(positions[2] + 180),
            positions[1] + WristConstants.intakeLengthM 
                * Math.sin(positions[2] + 180)};

        double pivotPosRad = cartesianToPolar(armPoint)[1];
        double telePosRad = cartesianToPolar(armPoint)[0];

        return null;
    }

    public double[] cartesianToPolar(double[] point) {
        return new double[] {
            Math.sqrt(point[0] + point[1]),
            Math.atan(point[1] / point[0])};
    }
}
