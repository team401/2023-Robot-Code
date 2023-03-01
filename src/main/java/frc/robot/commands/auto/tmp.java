package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class tmp extends CommandBase {

    private final PathPlannerTrajectory trajectory;
    private final Timer timer = new Timer();

    private final Field2d field = new Field2d();

    public tmp(PathPlannerTrajectory trajectoryPath) {

        trajectory = trajectoryPath;

        SmartDashboard.putData(field);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        field.setRobotPose(trajectory.sample(timer.get()).poseMeters);
    }
    
}
