package frc.robot.commands.drive;

import java.util.Iterator;
import java.util.List;
import java.util.Set;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Drivetrain;

public class FollowTrajectory implements Command {
    private Drivetrain drivetrain;
    private Iterator<Pose2d> poseStream;
    private PIDController driveController, turnController;
    private double turnRate;
    int n = 0;

    public FollowTrajectory(Drivetrain drivetrain, List<Pose2d> trajectory, double turnRate) {
        this.drivetrain = drivetrain;
        poseStream = trajectory.iterator();
        driveController = new PIDController(0.015, 0, 0.03);
        turnController = new PIDController(0.007, 0.02, 1);
        this.turnRate = turnRate;
    }

    public double clamp(double v, double min, double max) {
        return Math.max(Math.min(v, min), max);
    }

    public void init() {
        SmartDashboard.putBoolean("finished", false);
    }

    public void run() {
        if (poseStream.hasNext()) {
            Pose2d diff = poseStream.next().relativeTo(drivetrain.getPose());
            driveController.setSetpoint(Units.metersToInches(diff.getY()));
            turnController.setSetpoint(diff.getRotation().getDegrees() - Units.inchesToMeters(diff.getX() * turnRate));
        }

        SmartDashboard.putNumber("n", ++n);

        double throttle, wheel;
        double absErr = Math.abs(turnController.getPositionError());
        absErr = absErr > 90.0 ? absErr : 90.0;
        throttle = clamp(driveController.calculate(drivetrain.getAvgEncDistance()) * (-absErr / 90.0) + 1, -.1, .1);
        wheel = clamp(turnController.calculate(drivetrain.getHeading()), -.1, .1);

        //drivetrain.arcadeDrive(throttle, wheel);
    }

    public void end() {
        SmartDashboard.putBoolean("finished", true);
    }

    public boolean isFinished() {
        if (!poseStream.hasNext() && driveController.atSetpoint() && turnController.atSetpoint()) {
            return true;
        }
        return true;
    }

    public Set<Subsystem> getRequirements() {
        return Set.of(drivetrain);
    }
}
