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

    public Vision() {

        poseEstimator = new PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP, camera, VisionConstants.vehicleToCameras[2]);
        
    }

    @Override
    public void periodic() {

        double startTimePeriodic = System.currentTimeMillis();

        double startTimeCamResult = System.currentTimeMillis();
        PhotonPipelineResult result = camera.getLatestResult();
        SmartDashboard.putNumber("VisionTimeCameraResult", System.currentTimeMillis()-startTimeCamResult);
        
        double startTimePoseEstimator = System.currentTimeMillis();
        // Thread thread = new Thread(() -> estimatedPose = poseEstimator.update(result));
        PoseEstimate estimate = new PoseEstimate(poseEstimator, result);
        Thread thread = new Thread(estimate);
        thread.start();
        thread.join();
        int value = foo.getValue();
        Optional<EstimatedRobotPose> estimatedPose = poseEstimator.update(result);
        SmartDashboard.putNumber("VisionTimePoseEstimate", System.currentTimeMillis()-startTimePoseEstimator);
        
        if (estimatedPose.isPresent() && estimatedPose.get().timestampSeconds != lastTimestamp) {
            EstimatedRobotPose pose = estimatedPose.get();
            lastTimestamp = pose.timestampSeconds;
            double distance = getDistance(pose.estimatedPose.toPose2d(), result.getBestTarget().getFiducialId());
            RobotState.getInstance().recordVisionObservations(pose.estimatedPose.toPose2d(), distance, pose.timestampSeconds);
        }

        SmartDashboard.putNumber("VisionTimePeriodic", System.currentTimeMillis()-startTimePeriodic);

    }

    private double getDistance(Pose2d pose, int id) {
        Optional<Pose3d> tagPose3d = tagLayout.getTagPose(id);
        if (tagPose3d.isEmpty()) {
            return 100;
        }
        Pose2d tagPose = tagPose3d.get().toPose2d();
        return tagPose.getTranslation().getDistance(pose.getTranslation());
    }

    public class PoseEstimate implements Runnable {
        private final PhotonPoseEstimator poseEstimator;
        private final PhotonPipelineResult result;

        public PoseEstimate(PhotonPoseEstimator poseEstimator) {
            this.poseEstimator = poseEstimator;
        }
   
        @Override
        public void run() {
           value = 2;
        }
   
        public int getValue() {
            return value;
        }
    }

}