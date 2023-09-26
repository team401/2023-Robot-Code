package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;
import frc.robot.Constants.VisionConstants;

public class Camera {

    private static final AprilTagFieldLayout tagLayout = new AprilTagFieldLayout(VisionConstants.tags, VisionConstants.fieldLength, VisionConstants.fieldWidth);

    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;

    private volatile boolean hasNewPose = false;
    private volatile Pose3d calculatedPose = new Pose3d();
    private volatile Matrix<N3, N1> stdDevs = VecBuilder.fill(1000, 1000, 1000);
    private volatile double timestamp = 1;

    private String name;


    public Camera(String cameraName, Transform3d vehicleToCamera) {
        camera = new PhotonCamera(cameraName);
        poseEstimator = new PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP, camera, vehicleToCamera);
        poseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);

        name = cameraName;
    }

    public void periodic() {

        if (!camera.isConnected()) return;

        PhotonPipelineResult result = camera.getLatestResult(); 
        Optional<EstimatedRobotPose> estimatedPose = poseEstimator.update(result);
        if (estimatedPose.isEmpty()) return;
        EstimatedRobotPose estimation = estimatedPose.get();
        if (estimation.timestampSeconds == timestamp) return;
        if (estimation.targetsUsed.size() == 1 && 
            (estimation.targetsUsed.get(0).getPoseAmbiguity() > VisionConstants.singleTagAmbiguityCutoff || estimation.targetsUsed.get(0).getPoseAmbiguity() == -1))
            return;

        double distance = 0;
        for (PhotonTrackedTarget target : estimation.targetsUsed) {
            distance += target.getBestCameraToTarget().getTranslation().getDistance(new Translation3d());
        }

        distance /= estimation.targetsUsed.size();
        stdDevs = computeStdDevs(distance);

        calculatedPose = estimation.estimatedPose;
        timestamp = estimation.timestampSeconds;
        hasNewPose = true;

    }

    public boolean hasNewObservation() {
        return hasNewPose;
    }

    public void recordVisionObservation() {
        if(calculatedPose.toPose2d().getX() < 4 || calculatedPose.toPose2d().getX() > 12)
            RobotState.getInstance()
                .recordVisionObservations(calculatedPose.toPose2d(), stdDevs, timestamp);
        hasNewPose = false;

        log3dPose("Vision/" + name + "/RawPose", calculatedPose);
        // SmartDashboard.putNumber(camera.getName()+"X", calculatedPose.getX());
        // SmartDashboard.putNumber(camera.getName()+"Y", calculatedPose.getY());
        // SmartDashboard.putNumber(camera.getName()+"Theta", calculatedPose.getRotation().getRadians());
    }

    private Matrix<N3, N1> computeStdDevs(double distance) {
        double stdDev = Math.max(
            VisionConstants.minimumStdDev, 
            VisionConstants.stdDevEulerMultiplier * Math.exp(distance * VisionConstants.stdDevDistanceMultiplier)
        );
        return VecBuilder.fill(stdDev, stdDev, 1000);
    }

    /**
     * Logs a Pose3d in a format viewable in AdvantageScope
     */
    private void log3dPose(String key, Pose3d pose) {
        double[] doubles = new double[7];
        doubles[0] = pose.getX();
        doubles[1] = pose.getY();
        doubles[2] = pose.getZ();
        doubles[3] = pose.getRotation().getQuaternion().getW();
        doubles[4] = pose.getRotation().getQuaternion().getX();
        doubles[5] = pose.getRotation().getQuaternion().getY();
        doubles[6] = pose.getRotation().getQuaternion().getZ();
        SmartDashboard.putNumberArray(key, doubles);
    }
    
}