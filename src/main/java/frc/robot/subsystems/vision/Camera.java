package frc.robot.subsystems.vision;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;
import frc.robot.Constants.VisionConstants;

public class Camera {

    private static final AprilTagFieldLayout tagLayout = new AprilTagFieldLayout(VisionConstants.tags, VisionConstants.fieldLength, VisionConstants.fieldWidth);

    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;
    
    private volatile DoubleSupplier rotation;
    private volatile boolean cameraOn = true;

    private volatile boolean hasNewPose = false;
    private volatile Pose2d calculatedPose;
    private volatile double distance;
    private volatile double timestamp;

    public Camera(String cameraName, Transform3d vehicleToCamera, DoubleSupplier rotationSupplier) {
        
        camera = new PhotonCamera(cameraName);
        poseEstimator = new PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP, camera, vehicleToCamera);
        rotation = rotationSupplier;

    }

    public void periodic() {

        if (!camera.isConnected()) return;

        // boolean on = (Math.abs(rotation.getAsDouble()) > Math.PI / 2) ^ (poseEstimator.getRobotToCameraTransform().getX() > 0);
        // if (on != cameraOn)
        //     camera.setDriverMode(!on);
        // cameraOn = on;

        PhotonPipelineResult result = camera.getLatestResult(); 
        Optional<EstimatedRobotPose> estimatedPose = poseEstimator.update(result);
        
        if (estimatedPose.isPresent() && result.targets.size() > 1 && estimatedPose.get().timestampSeconds != timestamp) {
            hasNewPose = true;
            calculatedPose = estimatedPose.get().estimatedPose.toPose2d();
            distance = getDistance(estimatedPose.get().estimatedPose.toPose2d(), result.getBestTarget().getFiducialId());
            timestamp = estimatedPose.get().timestampSeconds;
        }

    }

    public boolean hasNewObservation() {
        return hasNewPose;
    }

    public void recordVisionObservation() {
        RobotState.getInstance().recordVisionObservations(calculatedPose, distance, timestamp);
        hasNewPose = false;
        SmartDashboard.putNumber(camera.getName()+"X", calculatedPose.getX());
        SmartDashboard.putNumber(camera.getName()+"Y", calculatedPose.getY());
        SmartDashboard.putNumber(camera.getName()+"Theta", calculatedPose.getRotation().getRadians());
    }

    private double getDistance(Pose2d pose, int id) {
        Optional<Pose3d> tagPose3d = tagLayout.getTagPose(id);
        if (tagPose3d.isEmpty()) {
            return 100;
        }
        Pose2d tagPose = tagPose3d.get().toPose2d();
        return tagPose.getTranslation().getDistance(pose.getTranslation());
    }
    
}