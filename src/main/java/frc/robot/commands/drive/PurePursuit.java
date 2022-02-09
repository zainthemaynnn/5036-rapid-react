package frc.robot.commands.drive;

import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Drivetrain;

public class PurePursuit implements Command {
    private Drivetrain drivetrain;
    private Trajectory trajectory;
    private static final int LOOKAHEAD = 1;
    private static final int LOOKAHEAD_MAX = 100;
    private static final int LOOKAHEAD_MIN = 0;

    public PurePursuit(Drivetrain drivetrain, Trajectory trajectory) {
        this.drivetrain = drivetrain;
        this.trajectory = trajectory;
    }

    private double toNthSpace(double... components) {
        int res = 0;
        for (double d : components) {
            res *= d;
        }
        return Math.pow(Math.abs(res), components.length);
    }

    public void run() {
        ChassisSpeeds speeds = drivetrain.getVelocity();
        Pose2d robotPose = drivetrain.getPose();
        double v = toNthSpace(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        double r = v * LOOKAHEAD;
    
        for (State state : trajectory.getStates()) {
            Pose2d pose = state.poseMeters;
            Pose2d rel = pose.relativeTo(robotPose);
            if (Math.pow(rel.getX(), 2) + Math.pow(rel.getY(), 2) <= Math.pow(r, 2)) {
                // TODO
                break;
            }
        }
    }

    public boolean isFinished() {
        return true;
    }

    public Set<Subsystem> getRequirements() {
        return Set.of(drivetrain);
    }
}
