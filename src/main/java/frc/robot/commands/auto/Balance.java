package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotState;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;

public class Balance extends CommandBase {

    private final Drive drive;

    private final PIDController rollController = new PIDController(AutoConstants.autoBalanceKp, AutoConstants.autoBalanceKi, AutoConstants.autoBalanceKd);

    private final PIDController yawController = new PIDController(DriveConstants.driveSnapKp, DriveConstants.driveSnapKi, DriveConstants.driveSnapKd);

    private final Timer onStationTimer = new Timer();
    private final Timer errorChecker = new Timer();
    private boolean onStationTimerStarted = false;

    public Balance(Drive drive) {
        this.drive = drive;

        yawController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drive);
    }

    @Override
    public void initialize() {

        onStationTimer.reset();
        onStationTimer.stop();
        onStationTimerStarted = false;

        errorChecker.reset();
        errorChecker.start();

        rollController.reset();
        yawController.reset();

    }

    @Override
    public void execute() {

        double omegaRadPerS = Math.min(yawController.calculate(drive.getRotation().getRadians(), 0), DriveConstants.maxTurnRate);
        omegaRadPerS = 0;

        double xMPerS = 0;

        if (!onStationTimer.hasElapsed(0.75)) {
            xMPerS = AutoConstants.initialBalanceSpeed;
            if (Math.abs(drive.getRoll()) > 0.2 && !onStationTimerStarted) {
                onStationTimer.reset();
                onStationTimer.start();
                onStationTimerStarted = true;
                errorChecker.reset();
                errorChecker.start();
            }
        }
        else {
            double output = -rollController.calculate(drive.getRoll(), 0);
            xMPerS = output;
            omegaRadPerS = 0;
        }

        ChassisSpeeds targetSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xMPerS, 0, omegaRadPerS, drive.getRotation());
        drive.setGoalChassisSpeeds(targetSpeeds);
        
    }

    @Override
    public boolean isFinished() {
        return errorChecker.hasElapsed(3);
    }

    @Override
    public void end(boolean isFinished) {
        drive.setGoalChassisSpeeds(new ChassisSpeeds(0, 0, 0));
    }
    
}
