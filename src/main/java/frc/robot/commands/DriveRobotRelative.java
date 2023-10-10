package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;

public class DriveRobotRelative extends CommandBase {
    private final Drive drive;
    private final DoubleSupplier xPercent;
    private final DoubleSupplier yPercent;
    private final DoubleSupplier omegaPercent;
    private final boolean fieldRelative;
    
    private double robotRotation;

    /** Creates a new DriveWithJoysticks. */
    public DriveRobotRelative(Drive drive, DoubleSupplier xPercent, DoubleSupplier yPercent, DoubleSupplier omegaPercent, boolean fieldRelative) {
        this.drive = drive;
        this.xPercent = xPercent;
        this.yPercent = yPercent;
        this.omegaPercent = omegaPercent;
        this.fieldRelative = fieldRelative;

        addRequirements(drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        robotRotation = drive.getRotation().getRadians();
        double modifier = Math.abs(robotRotation) < Math.PI ? -1 : 1;
        double xMPerS = modifier * (processJoystickInputs(xPercent.getAsDouble(), false) * DriveConstants.maxDriveSpeed) * Math.cos(robotRotation);
        double yMPerS = modifier * (processJoystickInputs(yPercent.getAsDouble(), false) * DriveConstants.maxDriveSpeed) * Math.sin(robotRotation);

        // double xMPerS = processJoystickInputs(xPercent.getAsDouble(), false) * DriveConstants.maxDriveSpeed;
        // double yMPerS = processJoystickInputs(yPercent.getAsDouble(), false) * DriveConstants.maxDriveSpeed;
        double omegaRadPerS = processJoystickInputs(omegaPercent.getAsDouble(), true) * DriveConstants.maxTurnRate;
        
        //Convert to field relative speeds
        ChassisSpeeds targetSpeeds = fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xMPerS, yMPerS, omegaRadPerS, drive.getRotation())
            : new ChassisSpeeds(xMPerS, yMPerS, omegaRadPerS);

        drive.setGoalChassisSpeeds(targetSpeeds);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    private double processJoystickInputs(double value, boolean square) {
        double scaledValue = 0.0;
        double deadband = DriveConstants.driveJoystickDeadbandPercent;
        if (Math.abs(value) > deadband) {
            scaledValue = (Math.abs(value) - deadband) / (1 - deadband);
            if (square) {
                scaledValue = Math.copySign(scaledValue * scaledValue, value);
            } else {
                scaledValue = Math.copySign(scaledValue, value);
            }
        }
        return scaledValue;
    }
}
