package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {

    //http://10.4.1.80:5800

    private final PhotonCamera camera = new PhotonCamera(VisionConstants.cameraNames[2]);
    
    private final AprilTagFieldLayout tagLayout = new AprilTagFieldLayout(VisionConstants.tags, 16.54175, 8.0137);

    private final PhotonPoseEstimator poseEstimator;
    
    private double lastTimestamp = 0;

    private final Notifier notifier = new Notifier(() -> periodicThread());

    private volatile boolean validPose;
    private volatile Pose2d calculatedPose;
    private volatile double distance;
    private volatile double timestamp;

    public Vision() {

        poseEstimator = new PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP, camera, VisionConstants.vehicleToCameras[2]);

        notifier.startPeriodic(0.02);
        
    }

    @Override
    public void periodic() {

        if (validPose) {
            validPose = false;
            RobotState.getInstance().recordVisionObservations(calculatedPose, distance, timestamp);
            SmartDashboard.putNumber("VisionX", calculatedPose.getX());
            SmartDashboard.putNumber("VisionY", calculatedPose.getY());
        }

    }

    private void periodicThread() {

        if (!camera.isConnected()) return;

        PhotonPipelineResult result = camera.getLatestResult();
        
        Optional<EstimatedRobotPose> estimatedPose = poseEstimator.update(result);
        
        if (estimatedPose.isPresent() && estimatedPose.get().timestampSeconds != lastTimestamp) {
            validPose = true;
            EstimatedRobotPose pose = estimatedPose.get();
            lastTimestamp = pose.timestampSeconds;
            calculatedPose = pose.estimatedPose.toPose2d();
            distance = getDistance(pose.estimatedPose.toPose2d(), result.getBestTarget().getFiducialId());
            timestamp = pose.timestampSeconds;
        }
        else {
            validPose = false;
        }

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