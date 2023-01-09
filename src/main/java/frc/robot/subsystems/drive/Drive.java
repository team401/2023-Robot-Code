package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.DriveConstants;

public class Drive extends SubsystemBase {

    private DriveModule[] modules = new DriveModule[4];
    private DriveAngle angle;
    private PIDController[] rotationPIDs = new PIDController[4];
    private SwerveModuleState[] goalModuleStates = new SwerveModuleState[4];
    private SwerveModulePosition[] currentModulePositions = new SwerveModulePosition[4];
    private SwerveDriveOdometry odometry;

    // If true, modules will run velocity control from the setpoint velocities in
    // moduleStates
    // If false, modules will not run velocity control from the setpoint velocities
    // in moduleStates,
    // and module drive motors will not be commanded to do anything. This allows
    // other setters,
    // such as the "setDriveVoltages" to have control of the modules.
    private boolean velocityControlEnabled = false;

    public Drive() {

        modules[0] = new DriveModule(CANDevices.frontLeftDriveMotorID, CANDevices.frontLeftRotationMotorID,
                CANDevices.frontLeftRotationEncoderID, DriveConstants.frontLeftAngleOffset);
        modules[1] = new DriveModule(CANDevices.frontRightDriveMotorID, CANDevices.frontRightRotationMotorID,
                CANDevices.frontRightRotationEncoderID, DriveConstants.frontRightAngleOffset);
        modules[2] = new DriveModule(CANDevices.backLeftDriveMotorID, CANDevices.backLeftRotationMotorID,
                CANDevices.backLeftRotationEncoderID, DriveConstants.backLeftAngleOffset);
        modules[3] = new DriveModule(CANDevices.backRightDriveMotorID, CANDevices.backRightRotationMotorID,
                CANDevices.backRightRotationEncoderID, DriveConstants.backRightAngleOffset);
        
        angle = new DriveAngle();

        for (int i = 0; i < 4; i++) {
            rotationPIDs[i] = new PIDController(DriveConstants.rotationKp, 0, DriveConstants.rotationKd);
            rotationPIDs[i].enableContinuousInput(-Math.PI, Math.PI);
            goalModuleStates[i] = new SwerveModuleState();
            currentModulePositions[i] = new SwerveModulePosition();
        }

        for (DriveModule module : modules) {
            module.zeroEncoders();
        }

        angle.resetHeading();

        odometry = new SwerveDriveOdometry(DriveConstants.kinematics, new Rotation2d(), currentModulePositions, new Pose2d());

    }

    @Override
    public void periodic() {
        // Display match time for driver
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());

        // Read inputs from module IO layers and tell the logger about them
        for (int i = 0; i < 4; i++) {
            modules[i].updateVariables();
        }
        angle.updateHeading();

        // Update odometry and report to RobotState
        Rotation2d headingRotation = new Rotation2d(MathUtil.angleModulus(angle.headingRad));
        SwerveModuleState[] measuredStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            measuredStates[i] = new SwerveModuleState(modules[i].driveVelocityRadPerS * DriveConstants.wheelRadiusM,
                    new Rotation2d(MathUtil.angleModulus(modules[i].rotationPositionRad)));
            currentModulePositions[i].distanceMeters = modules[i].drivePositionRad * DriveConstants.wheelRadiusM;
            currentModulePositions[i].angle = new Rotation2d(MathUtil.angleModulus(modules[i].rotationPositionRad));
        }

        // Optimize and set setpoints for each individual module
        for (int i = 0; i < 4; i++) {
            // Wrap encoder value to be within -pi, pi radians
            Rotation2d moduleRotation = new Rotation2d(MathUtil.angleModulus(modules[i].rotationPositionRad));

            // Optimize each module state
            SwerveModuleState optimizedState = SwerveModuleState.optimize(goalModuleStates[i], moduleRotation);
            double rotationSetpointRadians = optimizedState.angle.getRadians();
            double speedSetpointMPerS = optimizedState.speedMetersPerSecond;

            // Set module speed
            if (velocityControlEnabled) {
                double speedRadPerS = speedSetpointMPerS / DriveConstants.wheelRadiusM;
                double ffVolts = DriveConstants.driveModel.calculate(speedRadPerS);

                modules[i].setDriveVelocity(speedRadPerS, ffVolts);
            }

            // Set module rotation
            double rotationVoltage = rotationPIDs[i].calculate(rotationSetpointRadians, moduleRotation.getRadians());
            modules[i].setRotationVoltage(rotationVoltage);
        }
    }

    /**
     * Sets the target module states for each module. This can be used to
     * individually control each module.
     * 
     * @param states The array of states for each module
     */
    public void setGoalModuleStates(SwerveModuleState[] states) {
        velocityControlEnabled = true;
        goalModuleStates = states;
    }

    /**
     * Sets the raw voltages of the module drive motors. Heading is still set from
     * the angles set in
     * setGoalModuleStates. Note that this method disables velocity control until
     * setModuleStates or
     * setGoalChassisSpeeds is called again.
     * 
     * The primary use of this is to characterize the drive.
     * 
     * @param voltages The array of voltages to set in the modules
     */
    public void setDriveVoltages(double[] voltages) {
        velocityControlEnabled = false;
        for (int i = 0; i < 4; i++) {
            modules[i].setDriveVoltage(voltages[i]);
        }
    }

    /**
     * Sets the goal chassis speeds for the entire chassis.
     * 
     * @param speeds The target speed for the chassis
     */
    public void setGoalChassisSpeeds(ChassisSpeeds speeds) {
        SwerveModuleState[] moduleStates = DriveConstants.kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DriveConstants.maxSpeedMPerS);

        setGoalModuleStates(moduleStates);
    }

    /**
     * Returns the average angular speed of each wheel. This is used to characterize
     * the drive.
     * 
     * @return The average speed of each module in rad/s
     */
    public double getAverageSpeedRadPerS() {
        double sum = 0;
        for (int i = 0; i < 4; i++) {
            sum += modules[i].driveVelocityRadPerS;
        }
        return sum / 4.0;
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public Rotation2d getRotation() {
        return new Rotation2d(MathUtil.angleModulus(angle.headingRad));
    }

    public void resetHeading() {
        angle.resetHeading();
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(new Rotation2d(MathUtil.angleModulus(angle.headingRad)), currentModulePositions, pose);
    }

}
