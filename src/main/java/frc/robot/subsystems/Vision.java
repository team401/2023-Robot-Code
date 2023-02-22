package frc.robot.subsystems;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.Constants.VisionConstants;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {

    // private final PhotonCamera frontCamera = new PhotonCamera("????");
    private final PhotonCamera backCamera = new PhotonCamera("Arducam_OV9281_USB_Camera");

    private final Field2d visionField = new Field2d();

    private final MedianFilter filterX = new MedianFilter(5);
    private final MedianFilter filterY = new MedianFilter(5);
    private final MedianFilter filterTheta = new MedianFilter(5);

    private int nulled = 0;

    public Vision() {
        SmartDashboard.putData(visionField);
    }

    @Override
    public void periodic() {

        // Back camera
        PhotonPipelineResult cameraResult = backCamera.getLatestResult();
        Pose2d fieldToVehicle = calculateFieldToVehicle(cameraResult.getTargets(), VisionConstants.vehicleToBackCamera);
        if (fieldToVehicle != null) {
            double x = filterX.calculate(fieldToVehicle.getX());
            double y = filterY.calculate(fieldToVehicle.getY());
            double theta = filterTheta.calculate(fieldToVehicle.getRotation().getRadians());
            fieldToVehicle = new Pose2d(x, y, new Rotation2d(theta));
            RobotState.getInstance().recordVisionObservations(fieldToVehicle, cameraResult.getLatencyMillis() / 1000);
            // visionField.setRobotPose(fieldToVehicle);
        }
        else {
            nulled++;
        }
        SmartDashboard.putNumber("Nulled", nulled);

    }

    private Pose2d calculateFieldToVehicle(List<PhotonTrackedTarget> targets, Transform3d vehicleToCamera) {

        if (targets == null || targets.size() == 0 || targets.get(0) == null) return null;

        ArrayList<Pair<Pose2d, Double>> posesList = new ArrayList<Pair<Pose2d, Double>>();

        for (PhotonTrackedTarget target : targets) {

            Pose3d fieldToTag = VisionConstants.tagMap.get(target.getFiducialId());
            if (fieldToTag == null) continue;
    
            Transform3d cameraToTag = target.getBestCameraToTarget();
            if (cameraToTag == null) continue;
    
            Pose3d fieldToCamera = fieldToTag.transformBy(cameraToTag.inverse());
            Pose3d fieldToVehicle = fieldToCamera.transformBy(vehicleToCamera.inverse());

            posesList.add(new Pair<Pose2d, Double>(fieldToVehicle.toPose2d(), target.getPoseAmbiguity()));

        }

        double totalAmbiguity = 0;
        Translation2d translation = new Translation2d();
        Rotation2d rotation = new Rotation2d();

        int i = 0;
        for (Pair<Pose2d, Double> pair : posesList) {

            if (pair.getSecond() == 0) {
                return pair.getFirst();
            }

            totalAmbiguity += 1 - pair.getSecond();

            Translation2d t = pair.getFirst().getTranslation().times(1 - pair.getSecond());
            translation = new Translation2d(translation.getX()+t.getX(), translation.getY()+t.getY());
            Rotation2d r = pair.getFirst().getRotation().times(1 - pair.getSecond());
            rotation = new Rotation2d(rotation.getRadians() + r.getRadians());

            SmartDashboard.putString("Pose"+ i, pair.getFirst().toString());
            SmartDashboard.putString("Translation"+ i, pair.getFirst().getTranslation().times(1 - pair.getSecond()).toString());
            SmartDashboard.putString("Ambiguity"+ i++, pair.getSecond().toString());

        }

        SmartDashboard.putNumber("TotalAmbiguity", totalAmbiguity);
        SmartDashboard.putString("Translation", translation.toString());
        SmartDashboard.putNumber("Rotation", rotation.div(totalAmbiguity).getRadians());

        if (totalAmbiguity == 0) return null;
        return new Pose2d(translation.div(totalAmbiguity), rotation.div(totalAmbiguity));

    }
    
}

/*
    private Pose2d calculateFieldToVehicle(PhotonTrackedTarget target, Pose2d vehicleToCamera) {

        if (target == null) return null;
        Transform3d cameraToTarget3d = target.getBestCameraToTarget();
        Transform2d cameraToTarget2d = new Transform2d(cameraToTarget3d.getTranslation().toTranslation2d(), new Rotation2d(cameraToTarget3d.getRotation().getAngle()));
        if (cameraToTarget2d.getX() > 3) return null;
        Pose2d cameraToTarget = new Pose2d(cameraToTarget2d.getTranslation(), cameraToTarget2d.getRotation());

        Pose2d targetToField = VisionConstants.tagMap.get(target.getFiducialId());
        if (targetToField == null) return null;

        Pose2d fieldToCamera = poseInverse(cameraToTarget.transformBy(poseToTransform(targetToField)));
        Pose2d fieldToVehicle = fieldToCamera.transformBy(poseToTransform(poseInverse(vehicleToCamera)));

        if (target.getPoseAmbiguity() < 0.2 && target.getPoseAmbiguity() != -1)
            return fieldToVehicle;
        return null;

    }
 */