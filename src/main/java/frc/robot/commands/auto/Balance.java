package frc.robot.commands.auto;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotState;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;

public class Balance extends CommandBase {

    private enum Steps {
        DRIVE,
        BALANCE,
        DONE,
    }

    private final Drive drive;

    private Steps step = Steps.DRIVE;

    private Alliance alliance;

    private PIDController xController = 
        new PIDController(2.3, 0.01, 0);

    private PIDController thetaController = 
        new PIDController(DriveConstants.poseMoveRotationkP, 0, 0);


    private Timer timer = new Timer();

    private double xOffset = 0.0;

    public Balance(Drive drive) {
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        alliance = DriverStation.getAlliance();

        step = Steps.DRIVE;

        timer.start();
        timer.reset();

        xOffset = 0.0;
    }

    @Override
    public void execute() {
        if (step == Steps.DRIVE) {
            drive.setGoalChassisSpeeds(new ChassisSpeeds(-1.25, 0, 0));

            if (timer.get() > 2.2) {
                step = Steps.BALANCE;
            }
        }
        if (step == Steps.BALANCE) {
            timer.reset();

            double pitchFeedforward = 0.0;

            if (Math.abs(drive.getRoll()) > 0.1) {
                pitchFeedforward = 0.3;
            } else {
                pitchFeedforward = 0.0;
            }

            ChassisSpeeds speeds = new ChassisSpeeds(
                xController.calculate(RobotState.getInstance().getFieldToVehicle().getX(), 3.9)
                    + pitchFeedforward,
                0,
                thetaController.calculate(RobotState.getInstance().getFieldToVehicle().getRotation().getRadians(), 0.0)
            );

            drive.setGoalChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation()));
        }
    }

    @Override
    public boolean isFinished() {
        return step == Steps.DONE;
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
    
}
