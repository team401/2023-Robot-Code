package frc.robot.commands.auto;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotState;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;

public class Balance extends CommandBase {

    private enum Steps {
        DRIVE,
        BALANCE
    }

    private final Drive drive;

    private Steps step = Steps.DRIVE;

    private Alliance alliance;

    private PIDController xController = 
        new PIDController(DriveConstants.poseMoveTranslationkP, 0, 0);

    private PIDController thetaController = 
        new PIDController(DriveConstants.poseMoveRotationkP, 0, 0);


    private Timer timer = new Timer();

    public Balance(Drive drive) {
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        alliance = DriverStation.getAlliance();

        timer.start();
        timer.reset();
    }

    @Override
    public void execute() {
        if (step == Steps.DRIVE) {
            drive.setGoalChassisSpeeds(new ChassisSpeeds(-1.25, 0, 0));

            if (timer.get() > 2.5) {
                step = Steps.BALANCE;
            }
        }
        if (step == Steps.BALANCE) {
            timer.reset();

            ChassisSpeeds speeds = new ChassisSpeeds(
                xController.calculate(RobotState.getInstance().getFieldToVehicle().getX()),
                0,
                thetaController.calculate(RobotState.getInstance().getFieldToVehicle().getRotation().getRadians())
            );

            drive.setGoalChassisSpeeds(speeds);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
    
}
