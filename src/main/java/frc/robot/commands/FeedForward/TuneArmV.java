package frc.robot.commands.FeedForward;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class TuneArmV extends CommandBase {
    private ArmSubsystem arm;

    private double volts;

    private ArrayList<Double> velocities;

    double startPosition; // TODO

    double kS;
    double pastkV;
    double average = 0;
    double vel = 0;

    public TuneArmV(ArmSubsystem arm, double volts, double kS, double past) {
        this.arm = arm;
        this.volts = volts;
        this.kS = kS;
        this.pastkV = past;
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("Ended", false);
        arm.setVolts(volts);
        velocities = new ArrayList<Double>();
    }

    @Override
    public void execute() {
        vel = arm.getVelRadS();
        SmartDashboard.putNumber("Velocity", vel);
        if(Math.abs(arm.getPositionRad()) < 0.6) {
            velocities.add(vel);
        }
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("Ended", true);
        arm.stop();

        for(double v : velocities) {
            average += v;
        }

        average /= velocities.size();

        SmartDashboard.putNumber("kV", ((volts - kS) / average) + pastkV);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(arm.getPositionRad()) > 1;
    }
}
