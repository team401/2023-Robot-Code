package frc.robot.subsystems.vision;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {

    private final Camera[] cameras;

    private final Notifier notifier;

    private volatile DoubleSupplier rotation = () -> RobotState.getInstance().getFieldToVehicle().getRotation().getRadians();

    public Vision() {

        cameras = new Camera[] {
            // new Camera(VisionConstants.cameraNames[0], VisionConstants.vehicleToCameras[0], rotation),
            // new Camera(VisionConstants.cameraNames[1], VisionConstants.vehicleToCameras[1], rotation),
            new Camera(VisionConstants.cameraNames[2], VisionConstants.vehicleToCameras[2], rotation)
            // new Camera(VisionConstants.cameraNames[3], VisionConstants.vehicleToCameras[3], rotation)
        };

        notifier = new Notifier(() -> {
            for (int i = 0; i < 1; i++) {
                cameras[i].periodic();
            }
        });
        notifier.startPeriodic(0.02);

    }

    @Override
    public void periodic() {

        for (int i = 0; i < 1; i++) {
            if (cameras[i].hasNewObservation()) {
                cameras[i].recordVisionObservation();
            }
        }

    }

}