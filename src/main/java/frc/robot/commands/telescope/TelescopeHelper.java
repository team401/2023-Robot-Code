package frc.robot.commands.telescope;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TelescopeHelper {
    private TrapezoidProfile profile1;
    private TrapezoidProfile profile2;

    private boolean skipProfile2 = false;

    private double totalTime;
    private double delay;

    public TelescopeHelper(State start, State mid, State end, Constraints constraints, double totalTime) {

        this.totalTime = totalTime;

        double inside = (-4*end.position + 4*start.position) / (2*totalTime) + (totalTime/2);
        double midpoint = (-constraints.maxAcceleration / 4) * Math.pow(inside, 2) + start.position;
        midpoint = Math.max(0.05, midpoint);
        mid = new State(midpoint, 0);

        if (midpoint >= start.position || midpoint >= end.position) {
            skipProfile2 = true;
            mid = end;
        }
        
        profile1 = new TrapezoidProfile(constraints, mid, start);
        profile2 = new TrapezoidProfile(constraints, end, mid);

        delay = totalTime - (profile1.totalTime() + profile2.totalTime());
        if (delay < 0) {
            skipProfile2 = true;
            profile1 = new TrapezoidProfile(constraints, end, start);
        }

        SmartDashboard.putNumber("MID", mid.position);

    }

    public State calculate(double t) {
        SmartDashboard.putNumber("Total Time", t > totalTime ? 1 : -1);
        if (skipProfile2 || !profile1.isFinished(t - delay)) {
            SmartDashboard.putNumber("Profile2", 0.1);
            return profile1.calculate(t);
        }
        SmartDashboard.putNumber("Profile2", 0.3);


        return profile2.calculate(t - profile1.totalTime() - delay);
    }

    public boolean isFinished(double t) {
        return skipProfile2 ? profile1.isFinished(t) : profile2.isFinished(t - profile1.totalTime() - delay);
    }
}