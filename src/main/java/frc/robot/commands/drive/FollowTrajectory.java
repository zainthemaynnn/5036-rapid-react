package frc.robot.commands.drive;

import java.util.Iterator;
import java.util.Set;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Drivetrain;

public class FollowTrajectory implements Command {
    private Drivetrain drivetrain;
    private Iterator<State> poseStream;
    private PIDController driveController, turnController;

    public FollowTrajectory(Drivetrain drivetrain, Trajectory trajectory) {
        this.drivetrain = drivetrain;
        poseStream = trajectory.getStates().iterator();
        // TODO: left this stuff on sohaib's laptop. whoops.
        driveController = new PIDController(0, 0, 0);
        turnController = new PIDController(0, 0, 0);
    }

    public void run() {
        if (poseStream.hasNext()) {
            Pose2d diff = poseStream.next().poseMeters.relativeTo(drivetrain.getPose());
            driveController.setSetpoint(Units.metersToInches(diff.getX()));
            turnController.setSetpoint(diff.getRotation().getDegrees());
        }

        double throttle, wheel;
        double absErr = Math.abs(turnController.getPositionError());
        absErr = absErr > 90.0 ? absErr : 90.0;
        throttle = driveController.calculate(drivetrain.getAvgEncDistance()) * (-absErr / 90.0) + 1;
        wheel = turnController.calculate(drivetrain.getHeading());

        drivetrain.arcadeDrive(throttle, wheel);
    }

    public boolean isFinished() {
        return !poseStream.hasNext() && driveController.atSetpoint() && turnController.atSetpoint();
    }

    public Set<Subsystem> getRequirements() {
        return Set.of(drivetrain);
    }
}
