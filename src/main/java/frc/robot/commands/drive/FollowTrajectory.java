package frc.robot.commands.drive;

import java.util.Iterator;
import java.util.List;
import java.util.Set;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.devtools.PIDTuner;
import frc.robot.subsystems.Drivetrain;

public class FollowTrajectory implements Command {
    private Drivetrain drivetrain;
    private Iterator<Pose2d> poseStream;
    private PIDController driveController, turnController;
    private double turnRate;
    private static final double MAX_TURN = 90.0;

    private Pose2d next = new Pose2d(), diff = new Pose2d();
    private ShuffleboardTab tab;
    private double throttle, wheel;

    public FollowTrajectory(Drivetrain drivetrain, List<Pose2d> path, double turnRate) {
        this.drivetrain = drivetrain;
        poseStream = path.iterator();
        driveController = new PIDTuner("Drive");//new PIDController(0.015, 0, 0.03);
        turnController = new PIDTuner("Turn");//new PIDController(0.007, 0.02, 1);
        turnController.setTolerance(3);

        tab = Shuffleboard.getTab("FollowTrajectory");
        tab.addNumber("X", () -> drivetrain.getPose().getX());
        tab.addNumber("Y", () -> drivetrain.getPose().getY());
        tab.addNumber("R", () -> drivetrain.getPose().getRotation().getDegrees());
        tab.addNumber("nX", () -> next.getX());
        tab.addNumber("nY", () -> next.getY());
        tab.addNumber("nR", () -> next.getRotation().getDegrees());
        tab.addNumber("dX", () -> diff.getX());
        tab.addNumber("dY", () -> diff.getY());
        tab.addNumber("dR", () -> diff.getRotation().getDegrees());
        tab.addNumber("Throttle", () -> throttle);
        tab.addNumber("Wheel", () -> wheel);
        tab.addBoolean("hasNext", poseStream::hasNext);
        this.turnRate = turnRate;
    }

    private double clamp(double v, double min, double max) {
        return Math.min(Math.max(v, min), max);
    }

    private boolean atSetpoint() {
        return driveController.atSetpoint() && turnController.atSetpoint();
    }

    @Override
    public void initialize() {
        drivetrain.resetOdometry(new Pose2d());
        driveController.setSetpoint(0);
        turnController.setSetpoint(0);
    }

    @Override
    public void execute() {
        if (poseStream.hasNext() && atSetpoint()) {
            next = poseStream.next();
            /*
              INITIAL: new Pose2d(0, 0, new Rotation2d(A))
              TARGET: new Pose2d(-10, 50, new Rotation2d(B))

              A    B  X    Y
              0    0  -10  +50
              90   0  +50  +10
              180  0  +10  -50
              270  0  -10  -50

              verdict: zero @ [N]; anti-clockwise
              why the hell would they make it like this?
            */

            diff = next.relativeTo(drivetrain.getPose());
            driveController.setSetpoint(diff.getY());
            turnController.setSetpoint(next.getRotation().getDegrees() - clamp(diff.getX() * turnRate, -MAX_TURN, MAX_TURN));
        }

        double absErr = clamp(Math.abs(turnController.getPositionError()), 0.0, 90.0);
        throttle = driveController.calculate(drivetrain.getEncAvg()) * (1 - absErr / 90.0);
        wheel = -turnController.calculate(drivetrain.getPose().getRotation().getDegrees());
        drivetrain.arcadeDrive(clamp(throttle, -.1, .1), clamp(wheel, -.1, .1));
    }

    @Override
    public boolean isFinished() {
        return !poseStream.hasNext() && atSetpoint();
    }

    public Set<Subsystem> getRequirements() {
        return Set.of(drivetrain);
    }
}
