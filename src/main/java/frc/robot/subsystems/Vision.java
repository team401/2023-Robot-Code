package frc.robot.subsystems;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

    
    private final LinearFilter filterX = LinearFilter.singlePoleIIR(0.1, 0.02);
    private final LinearFilter filterY = LinearFilter.singlePoleIIR(0.1, 0.02);
    
    private boolean hasTarget = false;
    private double x = 0;
    private double y = 0;


    public Vision() {
    }

    public boolean hasTarget() {
        return hasTarget;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }


    @Override
    public void periodic() {

        hasTarget = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1;
        if (hasTarget) {
            x = filterX.calculate(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0));
            y = filterY.calculate(NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0));
        }
        else {
            filterX.reset();
            filterY.reset();
            x = 0;
            y = 0;
        }

    }

}

/*
boolean targetFound = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1;

if (targetFound) {

    double[] data = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpiblue").getDoubleArray(new double[]{0});
    if (data.length > 1) {
        boolean sawBlueTag = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0) > 4;
        if ((sawBlueTag && DriverStation.getAlliance() == Alliance.Red) || (!sawBlueTag && DriverStation.getAlliance() == Alliance.Blue)) return;
        // if (sawBlueTag != sawBlueTagLast) {
        //     filterX.reset();
        //     filterY.reset();
        // }
        // sawBlueTagLast = sawBlueTag;
        double latency = data[6] / 1000;
        Pose3d fieldToCamera = new Pose3d(new Translation3d(data[0], data[1], data[2]), new Rotation3d(data[3], data[4], data[5]));
        Pose3d fieldToVehicle3d = fieldToCamera.transformBy(VisionConstants.vehicleToBackCamera.inverse());
        Pose2d fieldToVehicle = new Pose2d(
            new Translation2d(
                filterX.calculate(fieldToVehicle3d.getX()), 
                filterY.calculate(fieldToVehicle3d.getY())), 
            new Rotation2d()
        );

        String json = NetworkTableInstance.getDefault().getTable("limelight").getEntry("json").getString("");
//json.indexOf("fID") != json.lastIndexOf("fID") && (
        if (fieldToVehicle.getX() < 4 || fieldToVehicle.getX() > 12.5) {
            // RobotState.getInstance().recordVisionObservations(fieldToVehicle, latency);
            // SmartDashboard.putString("VisionField", fieldToVehicle.toString());
            // field.setRobotPose(fieldToVehicle);
        }
    }

}
 */