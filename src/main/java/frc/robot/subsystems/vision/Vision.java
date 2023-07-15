package frc.robot.subsystems.vision;

import java.util.List;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.Constants.VisionConstants;
import frc.robot.util.PoseEstimator.TimestampedVisionUpdate;

public class Vision extends SubsystemBase {

    private final Camera[] cameras;

    private final Notifier notifier;

    public Vision(Consumer<TimestampedVisionUpdate> observationConsumer) {
        cameras = new Camera[] {
            new Camera(VisionConstants.cameraNames[0], VisionConstants.vehicleToCameras[0], observationConsumer),
            // new Camera(VisionConstants.cameraNames[1], VisionConstants.vehicleToCameras[1]),
            new Camera(VisionConstants.cameraNames[2], VisionConstants.vehicleToCameras[2], observationConsumer),
            new Camera(VisionConstants.cameraNames[3], VisionConstants.vehicleToCameras[3], observationConsumer)
        };

        notifier = new Notifier(() -> {
            for (int i = 0; i < 3; i++) {
                cameras[i].periodic();
            }
        });
        notifier.startPeriodic(0.02);
    }

    @Override
    public void periodic() {
        for (int i = 0; i < 3; i++) {
            if (cameras[i].hasNewObservation()) {
                cameras[i].recordVisionObservation();
            }
        }
    }
}