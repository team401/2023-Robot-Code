package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.drive.Drive;

public class CenterTag extends CommandBase {

    private final Drive drive;
    private final Vision vision;

    private final PIDController xController = new PIDController(0.01, 0, 0);
    private final PIDController yController = new PIDController(0.01, 0, 0);
    private final PIDController thetaController = new PIDController(3.5, 0, 0);

    private final Timer timer = new Timer();
    private final Timer noTagTimer = new Timer();

    private final double xSetpoint = -13.5;
    private final double ySetpoint = 17;

    private final double xThreshold = 1.5;
    private final double yThreshold = 1.5;

    public CenterTag(Drive driveSubystem, Vision visionSystem) {

        drive = driveSubystem;
        vision = visionSystem;

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drive);

    }

    @Override
    public void initialize() {

        xController.reset();
        yController.reset();
        thetaController.reset();

        timer.reset();
        timer.start();

        noTagTimer.reset();
        noTagTimer.start();

    }

    @Override
    public void execute() {

        if (vision.hasTarget()) {
            double xMPerS = -Math.max(Math.min(xController.calculate(vision.getY()-ySetpoint, 0), 0.2), -0.2);
            double yMPerS = -Math.max(Math.min(yController.calculate(vision.getX()-xSetpoint, 0), 0.2), -0.2);
            double omegaRadPerS = Math.min(thetaController.calculate(drive.getRotation().getRadians(), 0), Math.PI/4);
    
            drive.setGoalChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(xMPerS, yMPerS, omegaRadPerS, drive.getRotation()));
        }
        else {
            drive.setGoalChassisSpeeds(new ChassisSpeeds(0, 0, 0));
        }


        if (Math.abs(vision.getX() - xSetpoint) > xThreshold || Math.abs(vision.getY() - ySetpoint) > yThreshold) {
            timer.reset();
            timer.start();
        }

        if (vision.hasTarget()) {
            noTagTimer.reset();
            noTagTimer.start();
        }

    }

    @Override
    public boolean isFinished() {
        return (timer.hasElapsed(0.5)) || noTagTimer.hasElapsed(0.5);
    }

    @Override
    public void end(boolean isInterrupted) {
        drive.setGoalChassisSpeeds(new ChassisSpeeds(0, 0, 0));
    }
    
}
