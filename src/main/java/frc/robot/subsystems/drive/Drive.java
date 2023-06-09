package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.AngleIO.AngleIOInputs;
import frc.robot.subsystems.drive.ModuleIO.ModuleIOInputs;
import frc.robot.RobotState;

/**
 * Subsystem that manages all things related to robot driving
 */
public class Drive extends SubsystemBase {

    private final ModuleIO[] driveModules = new ModuleIO[4];

    private final ModuleIOInputsAutoLogged[] driveInputs = new ModuleIOInputsAutoLogged[] {
        new ModuleIOInputsAutoLogged(), new ModuleIOInputsAutoLogged(),
        new ModuleIOInputsAutoLogged(), new ModuleIOInputsAutoLogged()};

    private final AngleIO driveAngle;

    private final AngleIOInputsAutoLogged driveAngleInputs = new AngleIOInputsAutoLogged();

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
    public Drive(AngleIO angle, ModuleIO flModule, ModuleIO frModule, ModuleIO blModule, ModuleIO brModule) {
        driveModules[0] = flModule;
        driveModules[1] = frModule;
        driveModules[2] = blModule;
        driveModules[3] = brModule;

        driveAngle = angle;


        for (int i = 0; i < 4; i++) {
            driveModules[i].zeroEncoders();
            modulePositions[i] = new SwerveModulePosition();
            goalModuleStates[i] = new SwerveModuleState();

            rotationPIDs[i] = new PIDController(DriveConstants.rotationKps[i], 0, DriveConstants.rotationKds[i]);
            rotationPIDs[i].enableContinuousInput(-Math.PI, Math.PI);
            driveModules[i].setDrivePD(DriveConstants.driveKps[i], DriveConstants.driveKds[i]);

            driveModules[i].updateInputs(driveInputs[i]);

            modulePositions[i].distanceMeters = driveInputs[i].drivePositionRad * DriveConstants.wheelRadiusM;
            modulePositions[i].angle = new Rotation2d(driveInputs[i].rotationPositionRad);
        }

        setGoalChassisSpeeds(new ChassisSpeeds(0, 0, 0));

        RobotState.getInstance().initializePoseEstimator(getRotation(), modulePositions);
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Angle", driveAngle.getHeading());

        // Driving
        for (int i = 0; i < 4; i++) {
            driveModules[i].updateInputs(driveInputs[i]);
            Logger.getInstance().processInputs("Drive/Module"+i, driveInputs[i]);

            driveAngle.updateInputs(driveAngleInputs);
            Logger.getInstance().processInputs("Drive/Angle", driveAngleInputs);

            // Get encoder value
            Rotation2d moduleRotation = new Rotation2d(driveInputs[i].rotationPositionRad);

            // Optimize each module state
            SwerveModuleState optimizedState = SwerveModuleState.optimize(goalModuleStates[i], moduleRotation);
            
            // Set module speed
            double speedSetpointMPerS = optimizedState.speedMetersPerSecond;

            Logger.getInstance().recordOutput("Drive/Module"+i+"/DesiredSpeedMeters", speedSetpointMPerS);
            Logger.getInstance().recordOutput("Drive/Module"+i+"/SpeedError", 
                Math.abs(speedSetpointMPerS - driveInputs[i].driveVelocityRadPerSec 
                    * DriveConstants.wheelRadiusM / DriveConstants.driveWheelGearReduction));
            Logger.getInstance().recordOutput("Drive/Module"+i+"/DriveVelocityMeters",
                driveInputs[i].driveVelocityRadPerSec 
                    * DriveConstants.wheelRadiusM / DriveConstants.driveWheelGearReduction);
            
            double speedRadPerS = speedSetpointMPerS / DriveConstants.wheelRadiusM;
            double ffVolts = DriveConstants.driveFF.calculate(speedRadPerS);
            driveModules[i].setDriveVelocity(speedRadPerS, ffVolts);

            // Set module rotation
            double rotationSetpointRadians = optimizedState.angle.getRadians();

            double rotationVoltage = rotationPIDs[i].calculate(moduleRotation.getRadians(), rotationSetpointRadians);
            driveModules[i].setRotationVoltage(rotationVoltage);
            Logger.getInstance().recordOutput("Drive/Module"+i+"/DesiredRotation", rotationSetpointRadians);
            Logger.getInstance().recordOutput("Drive/Module"+i+"/RotationError",
                MathUtil.angleModulus(rotationSetpointRadians-moduleRotation.getRadians()));
        }

        // Pose estimation
        for (int i = 0; i < 4; i++) {
            modulePositions[i].distanceMeters = driveInputs[i].drivePositionRad * DriveConstants.wheelRadiusM;
            modulePositions[i].angle = new Rotation2d(driveInputs[i].rotationPositionRad);
        }
        RobotState.getInstance().recordDriveObservations(getRotation(), modulePositions);

        Logger.getInstance().recordOutput("Drive/ModuleStates/Setpoints", goalModuleStates);

        //TODO: make less stupid
        SwerveModuleState[] measuredStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            measuredStates[i] = new SwerveModuleState(
                driveInputs[i].driveVelocityRadPerSec
                    * DriveConstants.wheelRadiusM / DriveConstants.driveWheelGearReduction,
                Rotation2d.fromRadians(driveInputs[i].rotationPositionRad));
        }
        Logger.getInstance().recordOutput("Drive/ModuleStates/Measured", measuredStates);

        Logger.getInstance().recordOutput("Drive/Velocity", getChassisSpeeds().vxMetersPerSecond);
    }

    /**
     * Set the desired state for each module (velocity and angle)
     * @param states an array of SwerveModuleStates representing the desired state for each swerve module [frontLeft, frontRight, backLeft, backRight]
     */
    public void setGoalModuleStates(SwerveModuleState[] states) {
        for (int i = 0; i < 4; i++) {
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

        // Don't rotate the wheels if you're not going anywhere
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
        return new Rotation2d(MathUtil.angleModulus(driveAngleInputs.yawRad));
    }

    /**
     * Sets the heading (apparent rotation) of the robot to zero
     */
    public void resetHeading() {
        driveAngle.resetYaw();
    }

    public void setHeading(double heading) {
        driveAngle.setYaw(heading);
    }

    /**
     * @return the roll of the gyro in radians
     */
    public double getRoll() {
        return driveAngleInputs.rollRad;
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
            states[i] = new SwerveModuleState(
                driveInputs[i].driveVelocityRadPerSec,
                new Rotation2d(driveInputs[i].rotationPositionRad));
        }
        return states;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return DriveConstants.kinematics.toChassisSpeeds(getModuleStates());
    }

    public void setBrakeMode(boolean braked) {
        for (int i = 0; i < 4; i++) {
            driveModules[i].setBrakeMode(braked);
        }
    }

    public void toggleKill(int i) {
        driveModules[i].toggleKill();
    }

}