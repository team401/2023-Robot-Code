package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.LockWheels;

public class FindDriveKs extends CommandBase {
    private Drive drive;

    private double kS;

    private double[] driveStartPositions;

    public FindDriveKs(Drive drive) {
        this.drive = drive;
    }

    @Override
    public void initialize() {
        kS = 0.0;

        drive.lockWheels(LockWheels.STRAIGHT);

        driveStartPositions = drive.getDriveEncoderPositions();
    }

    @Override
    public void execute() {
        drive.setPIDOverride(true);
        drive.setVolts(kS);

        SmartDashboard.putNumber("kS", kS);

        kS += 0.001;
    }

    @Override
    public boolean isFinished() {
        double averageDriveDelta = 0.0;
        double[] driveCurrentPositions = drive.getDriveEncoderPositions();

        for (int i = 0; i < 4; i++) {
            averageDriveDelta += Math.abs(driveStartPositions[i] - driveCurrentPositions[i]);
        }

        averageDriveDelta /= 4;

        return averageDriveDelta >= 0.03;
    }

    @Override
    public void end(boolean interrupted) {
        drive.setVolts(0);
        drive.setPIDOverride(false);
        drive.lockWheels(LockWheels.UNLOCKED);
    }
}
