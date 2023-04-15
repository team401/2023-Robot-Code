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
import edu.wpi.first.wpilibj.Timer;
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
            CANDevices.backLeftRotationEncoderID, DriveConstants.backLeftAngleOffset, false);
        driveModules[3] = new DriveModule(CANDevices.backRightDriveMotorID, CANDevices.backRightRotationMotorID,
            CANDevices.backRightRotationEncoderID, DriveConstants.backRightAngleOffset, true);

        for (int i = 0; i < 4; i++) {
            driveModules[i].zeroEncoders();
            modulePositions[i] = new SwerveModulePosition();
            goalModuleStates[i] = new SwerveModuleState();

            rotationPIDs[i] = new PIDController(DriveConstants.rotationKps[i], 0, DriveConstants.rotationKds[i]);
            rotationPIDs[i].enableContinuousInput(-Math.PI, Math.PI);
            driveModules[i].setDrivePD(DriveConstants.driveKps[i], DriveConstants.driveKds[i]);

            modulePositions[i].distanceMeters = driveModules[i].getDrivePosition() * DriveConstants.wheelRadiusM;
            modulePositions[i].angle = new Rotation2d(driveModules[i].getRotationPosition());
        }
        // driveModules[1].setDrivePD(DriveConstants.frontRightDriveKp, DriveConstants.frontRightDriveKp);

        setGoalChassisSpeeds(new ChassisSpeeds(0, 0, 0));

        RobotState.getInstance().initializePoseEstimator(getRotation(), modulePositions);

    }

    @Override
    public void periodic() {

        // SmartDashboard.putNumber("Angle", driveAngle.getHeading());

        // Driving
        for (int i = 0; i < 4; i++) {
            // Get encoder value
            Rotation2d moduleRotation = new Rotation2d(driveModules[i].getRotationPosition());

            // Optimize each module state
            SwerveModuleState optimizedState = SwerveModuleState.optimize(goalModuleStates[i], moduleRotation);
            double rotationSetpointRadians = optimizedState.angle.getRadians();
            double speedSetpointMPerS = optimizedState.speedMetersPerSecond;

            // Set module speed
            double speedRadPerS = speedSetpointMPerS / DriveConstants.wheelRadiusM;
            double ffVolts = DriveConstants.driveFF.calculate(speedRadPerS);
            // SmartDashboard.putNumber("DesiredSpeed"+i, speedSetpointMPerS);
            // SmartDashboard.putNumber("ActualSpeed"+i, driveModules[i].getDriveVelocityMPerS());
            // SmartDashboard.putNumber("ErrorSpeed"+i, Math.abs(driveModules[i].getDriveVelocityMPerS()-speedSetpointMPerS));
            // SmartDashboard.putNumber("DriveOutput"+i, speedRadPerS);
            driveModules[i].setDriveVelocity(speedRadPerS, ffVolts);

            // Set module rotation
            // rotationSetpointRadians = 0;
            double rotationVoltage = rotationPIDs[i].calculate(moduleRotation.getRadians(), rotationSetpointRadians);
            driveModules[i].setRotationVoltage(rotationVoltage);
            // SmartDashboard.putNumber("DesiredRot"+i, rotationSetpointRadians);
            // SmartDashboard.putNumber("ActualRot"+i, moduleRotation.getRadians());
            // SmartDashboard.putNumber("DriveRotError"+i, MathUtil.angleModulus(rotationSetpointRadians-moduleRotation.getRadians()));

            // SmartDashboard.putNumber("DesiredDriveAngle"+i, rotationSetpointRadians);
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

        SmartDashboard.putNumber("DriveVelocity", getChassisSpeeds().vxMetersPerSecond);

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
        speeds = new ChassisSpeeds(speeds.vxMetersPerSecond * (babyMode ? 0.2 : 1), speeds.vyMetersPerSecond * (babyMode ? 0.2 : 1), speeds.omegaRadiansPerSecond * (babyMode ? 0.1 : 1));
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

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = driveModules[i].getModuleState();
        }
        return states;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return DriveConstants.kinematics.toChassisSpeeds(getModuleStates());
    }

    public void setBrakeMode(boolean braked) {
        for (int i = 0; i < 4; i++) {
            driveModules[i].setBrake(braked);
        }
    }

}