package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotState;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;

public class Balance extends CommandBase {

    private final Drive drive;

    private final PIDController pitchController = new PIDController(AutoConstants.autoBalanceKp, AutoConstants.autoBalanceKi, AutoConstants.autoBalanceKd);

    private double translationDirection = 1;

    private final PIDController yawController = new PIDController(DriveConstants.driveSnapKp, DriveConstants.driveSnapKi, DriveConstants.driveSnapKd);
    private double yawGoal = 0;

    private final Timer onStationTimer = new Timer();
    private boolean onStationTimerStarted = false;

    public Balance(Drive drive) {
        this.drive = drive;

        yawController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drive);
    }

    @Override
    public void initialize() {

        if (Math.abs(drive.getRotation().getRadians()) > Math.PI / 2) {
            yawGoal = Math.PI;
        }

        double posX = RobotState.getInstance().getFieldToVehicle().getX();
        if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
            translationDirection = posX < 4 ? 1 : -1;
        }
        else {
            translationDirection = posX < 12.5 ? 1 : -1;
        }

        onStationTimer.reset();
        onStationTimer.stop();
        onStationTimerStarted = false;

    }

    @Override
    public void execute() {

        double xMPerS = 0;

        if (!onStationTimer.hasElapsed(1.25)) {
            xMPerS = AutoConstants.initialBalanceSpeed * translationDirection;
            if (Math.abs(drive.getPitch()) > 0.25 && !onStationTimerStarted) {
                onStationTimer.reset();
                onStationTimer.start();
                onStationTimerStarted = true;
            }
        }
        else {
            double output = pitchController.calculate(drive.getPitch(), 0);
            xMPerS = output * translationDirection;
        }

        double omegaRadPerS = Math.min(yawController.calculate(drive.getRotation().getRadians(), yawGoal), DriveConstants.maxTurnRate);

        ChassisSpeeds targetSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xMPerS, 0, omegaRadPerS, drive.getRotation());
        drive.setGoalChassisSpeeds(targetSpeeds);
        
    }
    
}
