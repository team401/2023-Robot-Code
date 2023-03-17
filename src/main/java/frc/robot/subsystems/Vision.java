package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {

    //http://10.4.1.80:5800

    private final PhotonCamera[] cameras = {
        // new PhotonCamera(VisionConstants.cameraNames[0]),
        // new PhotonCamera(VisionConstants.cameraNames[1]),
        new PhotonCamera(VisionConstants.cameraNames[2])
        // new PhotonCamera(VisionConstants.cameraNames[3])
    };

    private final AprilTagFieldLayout tagLayout = new AprilTagFieldLayout(VisionConstants.tags, 16.54175, 8.0137);

    private final PhotonPoseEstimator[] poseEstimators;

    private final Field2d field = new Field2d();

    public Vision() {

        poseEstimators = new PhotonPoseEstimator[] {
            // new PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP, cameras[0], VisionConstants.vehicleToCameras[0]),
            // new PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP, cameras[1], VisionConstants.vehicleToCameras[1]),
            new PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP, cameras[0], VisionConstants.vehicleToCameras[0])
            // new PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP, cameras[3], VisionConstants.vehicleToCameras[3])
        };

        // SmartDashboard.putData(field);
        
    }


    @Override
    public void periodic() {

        Pose2d pose = new Pose2d(0, 0, new Rotation2d());

        int validPoseCount = 0;
        double latency = 0;

        for (int i = 0; i < 1; i++) {

            Optional<EstimatedRobotPose> estimatedPose = poseEstimators[i].update();
            if (estimatedPose.isPresent() && estimatedPose.get().targetsUsed.size() >= 2) {

                pose = new Pose2d(
                    pose.getX()+estimatedPose.get().estimatedPose.toPose2d().getX(), 
                    pose.getY()+estimatedPose.get().estimatedPose.toPose2d().getY(), 
                    new Rotation2d(pose.getRotation().getRadians()+estimatedPose.get().estimatedPose.toPose2d().getRotation().getRadians())
                );
                validPoseCount += 1;
                latency += cameras[i].getLatestResult().getLatencyMillis() / 1000;

            }

        }

        if (validPoseCount > 0) {
            latency /= validPoseCount;

            pose = new Pose2d(pose.getX()/validPoseCount, pose.getY()/validPoseCount, new Rotation2d(pose.getRotation().getRadians()/validPoseCount));

            RobotState.getInstance().recordVisionObservations(pose, latency);

            field.setRobotPose(pose);

        }

    }

}