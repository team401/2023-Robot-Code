// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

/**
 * This holonomic drive controller can be used to follow trajectories using a holonomic drivetrain
 * (i.e. swerve or mecanum). Holonomic trajectory following is a much simpler problem to solve
 * compared to skid-steer style drivetrains because it is possible to individually control forward,
 * sideways, and angular velocity.
 *
 * <p>The holonomic drive controller takes in one PID controller for each direction, forward and
 * sideways, and one profiled PID controller for the angular direction. Because the heading dynamics
 * are decoupled from translations, users can specify a custom heading that the drivetrain should
 * point toward. This heading reference is profiled for smoothness.
 */
//TODO: refactor all of this.
//  Where did this even come from?
@SuppressWarnings("unused")
public class CustomHolonomicDrive {
  private Pose2d m_poseError = new Pose2d();
  private Pose2d m_poseTolerance = new Pose2d(.05, .05, new Rotation2d());
  private double kControlFactorY = 0.2;
  private double kControlFactorX = 0.2;
  // private boolean m_enabled = true;

  private final PIDController m_xyController;
  private final PIDController m_zController;
  private final SlewRateLimiter m_xLimiter;
  private final SlewRateLimiter m_yLimiter;
  private final SlewRateLimiter m_zLimiter;

  /**
   * Constructs a holonomic drive controller.
   *
   * @param xyController A PID Controller to respond to error in the field-relative x direction.
   * @param zController A PID Controller to respond to error in the field-relative y direction.
   */
  @SuppressWarnings("ParameterName")
  public CustomHolonomicDrive(PIDController xyController, PIDController zController, SlewRateLimiter xLimiter,
      SlewRateLimiter yLimiter, SlewRateLimiter zLimiter) {
    m_xyController = xyController;
    m_zController = zController;
    m_zController.enableContinuousInput(0, 360);
    m_xLimiter = xLimiter;
    m_yLimiter = yLimiter;
    m_zLimiter = zLimiter;

  }

  /**
   * Returns true if the pose error is within tolerance of the reference.
   *
   * @return True if the pose error is within tolerance of the reference.
   */
  public boolean atReference() {
    final var eTranslate = m_poseError.getTranslation();
    final var tolTranslate = m_poseTolerance.getTranslation();
    return Math.abs(eTranslate.getX()) < tolTranslate.getX()
        && Math.abs(eTranslate.getY()) < tolTranslate.getY();
  }

  public double distanceFromXY(Pose2d currentPose, Pose2d poseRef) {
    return Math
        .sqrt(Math.pow(currentPose.getX() - poseRef.getX(), 2) + Math.pow(currentPose.getY() - poseRef.getY(), 2));
  }

  public Rotation2d distanceFromZ(Pose2d currentPose, Pose2d poseRef) {
    return Rotation2d
        .fromDegrees(Math.abs(currentPose.getRotation().getDegrees() - poseRef.getRotation().getDegrees()));
  }

  public double slewRateX(double xSpeed) {
    return m_xLimiter.calculate(xSpeed);
  }

  public double slewRateY(double ySpeed) {
    return m_yLimiter.calculate(ySpeed);
  }

  public double slewRateZ(double zSpeed) {
    return m_zLimiter.calculate(zSpeed);
  }

  /**
   * Sets the pose error which is considered tolerance for use with atReference().
   *
   * @param tolerance The pose error which is tolerable.
   */
  public void setTolerance(Pose2d tolerance) {
    m_poseTolerance = tolerance;
  }

  /**
   * Returns the next output of the holonomic drive controller.
   *
   * @param currentPose The current pose.
   * @param poseRef The desired pose.
   * @param linearVelocityRefMeters The linear velocity reference.
   * @param angleRef The angular reference.
   * @param angleVelocityRefRadians The angular velocity reference.
   * @return The next output of the holonomic drive controller.
   */
  @SuppressWarnings("LocalVariableName")
  public ChassisSpeeds calculateOnlyY(
      Pose2d currentPose,
      Pose2d poseRef,
      Supplier<Double> xSpeed,
      Supplier<Double> ySpeed,
      Supplier<Double> angleRef) {

    // Calculate feedforward velocities (field-relative).
    // double xFF = linearVelocityRefMeters * poseRef.getRotation().getCos();
    // double yFF = linearVelocityRefMeters * poseRef.getRotation().getSin();
    // double thetaFF = angleVelocityRefRadians;
    m_poseError = poseRef.relativeTo(currentPose);

    // Calculate feedback velocities (based on position error).
    double xFeedback = m_xyController.calculate(currentPose.getX(), poseRef.getX());
    double yFeedback = m_xyController.calculate(currentPose.getY(), poseRef.getY());
    double thetaFeedback = m_zController.calculate(currentPose.getRotation().getDegrees(),
        poseRef.getRotation().getDegrees());

    // EricNubControls EricControls = new EricNubControls();
    // double x_speed = EricControls.addDeadzoneScaled(xSpeed.get(), 0.1);
    // double y_speed = EricControls.addDeadzoneScaled(ySpeed.get(), 0.1);

    // double x = xFeedback + (kControlFactorX * x_speed);
    // double y = yFeedback + (kControlFactorY * y_speed);
    double x = xFeedback;
    double y = yFeedback;

    double mag = Math.sqrt((x * x) + (y * y));

    // if you dont want to be at max speed, dont use this function
    // double xF = x / mag;
    // double yF = y / mag;

    // Calculate feedback velocities (based on angle error).

    // Return next output.
    // EricControls.addEricCurve(EricControls.addDeadzoneScaled(angleRef.get(), 0.1))
    return ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed.get() * Constants.DriveConstants.poseMoveTranslationMaxVel / 2,
        y * Constants.DriveConstants.poseMoveTranslationMaxVel,
        thetaFeedback
            * Constants.DriveConstants.poseMoveRotationMaxVel,
        currentPose.getRotation());
  }

  @SuppressWarnings("LocalVariableName")
  public ChassisSpeeds calculateOnlyX(
      Pose2d currentPose,
      Pose2d poseRef,
      Supplier<Double> xSpeed,
      Supplier<Double> ySpeed,
      Supplier<Double> angleRef) {

    // Calculate feedforward velocities (field-relative).
    // double xFF = linearVelocityRefMeters * poseRef.getRotation().getCos();
    // double yFF = linearVelocityRefMeters * poseRef.getRotation().getSin();
    // double thetaFF = angleVelocityRefRadians;
    m_poseError = poseRef.relativeTo(currentPose);

    // Calculate feedback velocities (based on position error).
    double xFeedback = m_xyController.calculate(currentPose.getX(), poseRef.getX());
    double yFeedback = m_xyController.calculate(currentPose.getY(), poseRef.getY());
    double thetaFeedback = m_zController.calculate(currentPose.getRotation().getDegrees(),
        poseRef.getRotation().getDegrees());

    // EricNubControls EricControls = new EricNubControls();
    // double x_speed = EricControls.addDeadzoneScaled(xSpeed.get(), 0.1);
    // double y_speed = EricControls.addDeadzoneScaled(ySpeed.get(), 0.1);

    // double x = xFeedback + (kControlFactorX * x_speed);
    // double y = yFeedback + (kControlFactorY * y_speed);
    double x = xFeedback;
    double y = yFeedback;

    double mag = Math.sqrt((x * x) + (y * y));

    // if you dont want to be at max speed, dont use this function
    // double xF = x / mag;
    // double yF = y / mag;

    // Calculate feedback velocities (based on angle error).

    // Return next output.
    // EricControls.addEricCurve(EricControls.addDeadzoneScaled(angleRef.get(), 0.1))
    return ChassisSpeeds.fromFieldRelativeSpeeds(x * DriveConstants.poseMoveTranslationMaxVel / 2,
        ySpeed.get() * DriveConstants.poseMoveTranslationMaxVel,
        thetaFeedback
            * DriveConstants.poseMoveRotationMaxVel,
        currentPose.getRotation());
  }

  @SuppressWarnings("LocalVariableName")
  public ChassisSpeeds calculate(
      Pose2d currentPose,
      Pose2d poseRef) {

    // Calculate feedforward velocities (field-relative).
    // double xFF = linearVelocityRefMeters * poseRef.getRotation().getCos();
    // double yFF = linearVelocityRefMeters * poseRef.getRotation().getSin();
    // double thetaFF = angleVelocityRefRadians;
    m_poseError = poseRef.relativeTo(currentPose);

    // Calculate feedback velocities (based on position error).
    double xFeedback = m_xyController.calculate(currentPose.getX(), poseRef.getX());
    double yFeedback = m_xyController.calculate(currentPose.getY(), poseRef.getY());
    double thetaFeedback = m_zController.calculate(currentPose.getRotation().getDegrees(),
        poseRef.getRotation().getDegrees());

    // EricNubControls EricControls = new EricNubControls();
    // double x_speed = EricControls.addDeadzoneScaled(xSpeed.get(), 0.1);
    // double y_speed = EricControls.addDeadzoneScaled(ySpeed.get(), 0.1);

    // double x = xFeedback + (kControlFactorX * x_speed);
    // double y = yFeedback + (kControlFactorY * y_speed);
    double x = xFeedback;
    double y = yFeedback;

    double mag = Math.sqrt((x * x) + (y * y));

    // if you dont want to be at max speed, dont use this function
    // double xF = x / mag;
    // double yF = y / mag;

    // Calculate feedback velocities (based on angle error).

    // Return next output.
    // EricControls.addEricCurve(EricControls.addDeadzoneScaled(angleRef.get(), 0.1))
    return ChassisSpeeds.fromFieldRelativeSpeeds(x * DriveConstants.poseMoveTranslationMaxVel,
        y * DriveConstants.poseMoveTranslationMaxVel,
        thetaFeedback
            * DriveConstants.poseMoveRotationMaxVel,
        currentPose.getRotation());
  }

  public ChassisSpeeds fullCalulate(Pose2d desPose2d, Pose2d curPose,
      ChassisSpeeds curChassisSpeeds) {
    // get the error between the current pose and the desired pose
    m_poseError = desPose2d.relativeTo(curPose);
    // get the error between the current chassis speeds and the desired chassis speeds
    ChassisSpeeds m_chassisSpeeds = this.calculate(curPose, desPose2d);
    // use slew rate limiter to limit the change in chassis speeds
    // m_chassisSpeeds.vxMetersPerSecond = m_xLimiter.calculate(m_chassisSpeeds.vxMetersPerSecond);
    // m_chassisSpeeds.vyMetersPerSecond = m_yLimiter.calculate(m_chassisSpeeds.vyMetersPerSecond);
    // m_chassisSpeeds.omegaRadiansPerSecond = m_zLimiter.calculate(m_chassisSpeeds.omegaRadiansPerSecond);

    // return the new chassis speeds
    return m_chassisSpeeds;
  }

  @SuppressWarnings("LocalVariableName")
  public ChassisSpeeds calculate(
      Pose2d currentPose,
      ExtendedPathPoint poseRef,
      Supplier<Double> xSpeed,
      Supplier<Double> ySpeed,
      Supplier<Double> angleRef, Boolean finalPoint) {

    // Calculate feedforward velocities (field-relative).
    // double xFF = linearVelocityRefMeters * poseRef.getRotation().getCos();
    // double yFF = linearVelocityRefMeters * poseRef.getRotation().getSin();
    // double thetaFF = angleVelocityRefRadians;
    Pose2d pose2dRef = poseRef.getPose2d();
    m_poseError = pose2dRef.relativeTo(currentPose);

    // Calculate feedback velocities (based on position error).
    double xFeedback = m_xyController.calculate(currentPose.getX(), pose2dRef.getX());
    double yFeedback = m_xyController.calculate(currentPose.getY(), pose2dRef.getY());
    double thetaFeedback = m_zController.calculate(currentPose.getRotation().getDegrees(),
        poseRef.getRotation().getDegrees());

    // EricNubControls EricControls = new EricNubControls();
    // double x_speed = EricControls.addDeadzoneScaled(xSpeed.get(), 0.1);
    // double y_speed = EricControls.addDeadzoneScaled(ySpeed.get(), 0.1);

    // double x = xFeedback + (kControlFactorX * x_speed);
    // double y = yFeedback + (kControlFactorY * y_speed);
    double x = xFeedback;
    double y = yFeedback;
    double xF = 0;
    double yF = 0;
    if (finalPoint) {
      xF = x;
      yF = y;
    } else {
      double mag = Math.sqrt((x * x) + (y * y));

      // if you dont want to be at max speed, dont use this function
      xF = x / mag;
      yF = y / mag;
    }

    // Calculate feedback velocities (based on angle error).

    // Return next output.
    // EricControls.addEricCurve(EricControls.addDeadzoneScaled(angleRef.get(), 0.1))
    return ChassisSpeeds.fromFieldRelativeSpeeds(xF * DriveConstants.poseMoveTranslationMaxVel,
        yF * DriveConstants.poseMoveTranslationMaxVel,
        thetaFeedback
            * DriveConstants.poseMoveRotationMaxVel,
        currentPose.getRotation());
  }
}
