package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {

    // private final PhotonCamera frontCamera = new PhotonCamera("????");
    private final PhotonCamera backCamera = new PhotonCamera("Arducam_OV9281_USB_Camera");

    private PhotonPipelineResult cameraResult;

    public Vision() {
    }

    public double getYaw(int id) {
        for (PhotonTrackedTarget target : cameraResult.targets) {
            if (target.getFiducialId() == id) {
                return target.getYaw() + 12.3;
            }
        }
        return -1;
    }
    
    public double getPitch(int id) {
        for (PhotonTrackedTarget target : cameraResult.targets) {
            if (target.getFiducialId() == id) {
                return target.getPitch() + 12.65;
            }
        }
        return -1;
    }

    @Override
    public void periodic() {

        cameraResult = backCamera.getLatestResult();

    }

}