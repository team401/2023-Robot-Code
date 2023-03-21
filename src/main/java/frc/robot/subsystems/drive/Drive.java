package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotState;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CANDevices;

/**
 * Subsystem that manages all things related to robot driving
 */
public class Drive extends SubsystemBase {

    /**
     * Used to set module states and get data for RobotState
     */
    private final DriveModule[] driveModules = new DriveModule[4];

    /**
     * Used to get the rotation of the robot
     */
    private final DriveAngle driveAngle = new DriveAngle();

    /**
     * The positions of each module used to update RobotState
     */
    private SwerveModulePosition modulePositions[] = new SwerveModulePosition[4];

    /**
     * The desired state for each swerve module
     */
    private SwerveModuleState[] goalModuleStates = new SwerveModuleState[4];

    /**
     * The controllers used to calculate the output volts for the rotation motors
     */
    private PIDController[] rotationPIDs = new PIDController[4];

    /**
     * True if drive should be slow, false if drive should be going at max speed
     */
    private boolean babyMode = false;

    /**
     * Initialize all the modules, data arrays, and RobotState
     */
    public Drive() {

        driveModules[0] = new DriveModule(CANDevices.frontLeftDriveMotorID, CANDevices.frontLeftRotationMotorID,
            CANDevices.frontLeftRotationEncoderID, DriveConstants.frontLeftAngleOffset, false);
        driveModules[1] = new DriveModule(CANDevices.frontRightDriveMotorID, CANDevices.frontRightRotationMotorID,
            CANDevices.frontRightRotationEncoderID, DriveConstants.frontRightAngleOffset, true);
        driveModules[2] = new DriveModule(CANDevices.backLeftDriveMotorID, CANDevices.backLeftRotationMotorID,
            CANDevices.backLeftRotationEncoderID, DriveConstants.backLeftAngleOffset, true);
        driveModules[3] = new DriveModule(CANDevices.backRightDriveMotorID, CANDevices.backRightRotationMotorID,
            CANDevices.backRightRotationEncoderID, DriveConstants.backRightAngleOffset, true);

        for (int i = 0; i < 4; i++) {
            driveModules[i].zeroEncoders();
            modulePositions[i] = new SwerveModulePosition();
            goalModuleStates[i] = new SwerveModuleState();

            rotationPIDs[i] = new PIDController(DriveConstants.rotationKp, 0, DriveConstants.rotationKd);
            rotationPIDs[i].enableContinuousInput(-Math.PI, Math.PI);
            driveModules[i].setDrivePD(DriveConstants.driveKp, DriveConstants.driveKd);

            modulePositions[i].distanceMeters = driveModules[i].getDrivePosition() * DriveConstants.wheelRadiusM;
            modulePositions[i].angle = new Rotation2d(driveModules[i].getRotationPosition());
        }

        for (int i = 0; i < 4; i++) {
            modulePositions[i].distanceMeters = driveModules[i].getDrivePosition() * DriveConstants.wheelRadiusM;
            modulePositions[i].angle = new Rotation2d(driveModules[i].getRotationPosition());
        }

        setGoalChassisSpeeds(new ChassisSpeeds(0, 0, 0));

        RobotState.getInstance().initializePoseEstimator(getRotation(), modulePositions);

    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Angle", driveAngle.getHeading());

        // Driving
        for (int i = 0; i < 4; i++) {
            // Wrap encoder value to be within -pi, pi radians
            Rotation2d moduleRotation = new Rotation2d(driveModules[i].getRotationPosition());

            // Optimize each module state
            SwerveModuleState optimizedState = SwerveModuleState.optimize(goalModuleStates[i], moduleRotation);
            double rotationSetpointRadians = optimizedState.angle.getRadians();
            double speedSetpointMPerS = optimizedState.speedMetersPerSecond;

            // Set module speed
            double speedRadPerS = speedSetpointMPerS / DriveConstants.wheelRadiusM;
            double ffVolts = DriveConstants.driveFF.calculate(speedRadPerS);
            // SmartDashboard.putNumber("DriveOutput"+i, speedRadPerS);
            driveModules[i].setDriveVelocity(speedRadPerS, ffVolts);
            // SmartDashboard.putNumber("DRiveOutputVolts"+i, driveModules[i].getDriveVoltageApplied());

            // Set module rotation
            double rotationVoltage = rotationPIDs[i].calculate(moduleRotation.getRadians(), rotationSetpointRadians);
            driveModules[i].setRotationVoltage(rotationVoltage);
        }

        // Pose estimation
        for (int i = 0; i < 4; i++) {
            modulePositions[i].distanceMeters = driveModules[i].getDrivePosition() * DriveConstants.wheelRadiusM;
            modulePositions[i].angle = new Rotation2d(driveModules[i].getRotationPosition());
            // SmartDashboard.putNumber("DriveAngle"+i, modulePositions[i].angle.getRadians());
            // SmartDashboard.putNumber("DriveStator"+i, driveModules[i].getDriveStatorCurrent());
            // SmartDashboard.putNumber("RotationStator"+i, driveModules[i].getRotationStatorCurrent());
        }
        RobotState.getInstance().recordDriveObservations(getRotation(), modulePositions);

        RobotState.getInstance().getFieldToVehicle();

        SmartDashboard.putNumber("DriveVelocity", getVelocity());

        // SmartDashboard.putNumber("Roll", driveAngle.getRoll());

    }

    /**
     * Set the desired state for each module (velocity and angle)
     * @param states an array of SwerveModuleStates representing the desired state for each swerve module [frontLeft, frontRight, backLeft, backRight]
     */
    public void setGoalModuleStates(SwerveModuleState[] states) {
        // SmartDashboard.putNumber("lol", System.currentTimeMillis());
        for (int i = 0; i < 4; i++) {
            // SmartDashboard.putNumber("lel"+i, states[i].speedMetersPerSecond);
            goalModuleStates[i] = states[i];
        }
    }

    /**
     * Set the desired velocities of the robot drive (xVelocity, yVelocity, omegaVelocity)
     * @param speeds the desired speed of the robot
     */
    public void setGoalChassisSpeeds(ChassisSpeeds speeds) {
        if (DriverStation.isAutonomousEnabled()) {
            speeds = new ChassisSpeeds(
                Math.max(Math.min(speeds.vxMetersPerSecond, AutoConstants.kMaxVelocityMetersPerSecond), -AutoConstants.kMaxVelocityMetersPerSecond), 
                Math.max(Math.min(speeds.vyMetersPerSecond, AutoConstants.kMaxVelocityMetersPerSecond), -AutoConstants.kMaxVelocityMetersPerSecond),
                speeds.omegaRadiansPerSecond
            );
        }
        speeds = new ChassisSpeeds(speeds.vxMetersPerSecond * (babyMode ? 0.2 : 1), speeds.vyMetersPerSecond * (babyMode ? 0.2 : 1), speeds.omegaRadiansPerSecond * (babyMode ? 0.2 : 1));
        SwerveModuleState[] goalModuleStates = DriveConstants.kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(goalModuleStates, DriveConstants.maxDriveSpeed);
        if (speeds.vxMetersPerSecond == 0 && speeds.vyMetersPerSecond == 0 && speeds.omegaRadiansPerSecond == 0) {
            goalModuleStates = new SwerveModuleState[] {
                new SwerveModuleState(0, modulePositions[0].angle), 
                new SwerveModuleState(0, modulePositions[1].angle),
                new SwerveModuleState(0, modulePositions[2].angle),
                new SwerveModuleState(0, modulePositions[3].angle)
            };
        }
        setGoalModuleStates(goalModuleStates);
    }


    /**
     * @return the rotation of the robot in radians from the gyro
     */
    public Rotation2d getRotation() {
        return new Rotation2d(MathUtil.angleModulus(driveAngle.getHeading()));
    }

    /**
     * Sets the heading (apparent rotation) of the robot to zero
     */
    public void resetHeading() {
        driveAngle.resetHeading();
    }

    public void setHeading(double heading) {
        driveAngle.setHeading(heading);
    }

    /**
     * @return the roll of the gyro in radians
     */
    public double getRoll() {
        return driveAngle.getRoll();
    }

    /**
     * @param baby true to make the drive slow, false to make it go max speeds
     */
    public void setBabyMode(boolean baby) {
        babyMode = baby;
    }

    /**
     * Used to pass the current field to vehicle to RobotState, needs to pass through drive because driveModulePositions and rotation are needed to set fieldToVehicle
     * @param fieldToVehicle the current fieldToVehicle pose
     */
    public void setFieldToVehicle(Pose2d fieldToVehicle) {
        RobotState.getInstance().setFieldToVehicle(getRotation(), modulePositions, fieldToVehicle);
    }

    public void setVolts(double v) {
        driveModules[0].setDriveVoltage(v);
        driveModules[1].setDriveVoltage(v);
        driveModules[2].setDriveVoltage(v);
        driveModules[3].setDriveVoltage(v);
    }

    public double getVelocity() {
        return (driveModules[0].getDriveVelocityMPerS() + driveModules[1].getDriveVelocityMPerS() + driveModules[2].getDriveVelocityMPerS() + driveModules[3].getDriveVelocityMPerS()) / 4;
    }

}