package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;

public class MeasureWheelRadius extends CommandBase {

    /**
     * Command to measure the wheel radius of the drive
     * For swerve, we assume each module wheel radius to be the same and use the average distance driven as the robot distance
     */

    private final Drive drive;

    private double volts = 0;

    public MeasureWheelRadius(double distanceFt, Drive subsystem) {

        drive = subsystem;

        addRequirements(drive);

    }

    @Override
    public void initialize() {

        volts = 0;

        drive.setGoalChassisSpeeds(new ChassisSpeeds(0.5, 0, 0));

    }

	@Override
	public void execute() {

        volts += 0.01;

        SmartDashboard.putNumber("DriveVolts", volts);

        SmartDashboard.putNumber("DriveVel", drive.getVelocity());

        // drive.setVolts(-3);

	}

    @Override
    public void end(boolean interrupted) {

        drive.setVolts(0);
    }

}