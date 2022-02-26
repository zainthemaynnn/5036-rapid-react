package frc.robot.commands.drive;

import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.FileSystem;
import java.util.Iterator;
import java.util.List;
import java.util.Set;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.devtools.PIDTuner;
import frc.robot.subsystems.Drivetrain;

public class FollowTrajectory implements Command {
    public static class PoseData {
        public Pose2d pose;
        public double turnRate, maxY;
        public boolean odomReset;
        public PoseData(Pose2d pose, double turnRate, double maxY, boolean odomReset) {
            this.pose = pose;
            this.turnRate = turnRate;
            this.maxY = maxY;
            this.odomReset = odomReset;
        }
    }

    private Drivetrain drivetrain;
    private Iterator<PoseData> poseStream;
    private PIDController driveController, turnController;
    private static final double MAX_TURN = 90.0;

    private Pose2d next = new Pose2d(), err = new Pose2d();
    private PoseData params;
    private ShuffleboardTab tab;
    private double throttle, wheel;
    private boolean finished = false;

    public FollowTrajectory(Drivetrain drivetrain, List<PoseData> path) {
        this.drivetrain = drivetrain;
        poseStream = path.iterator();
        driveController = new PIDTuner("Drive");//new PIDController(0.015, 0, 0.03);
        turnController = new PIDTuner("Turn");//new PIDController(0.007, 0.02, 1);

        tab = Shuffleboard.getTab("FollowTrajectory");
        tab.addNumber("X", () -> drivetrain.getPose().getX());
        tab.addNumber("Y", () -> drivetrain.getPose().getY());
        tab.addNumber("R", () -> drivetrain.getPose().getRotation().getDegrees());
        tab.addNumber("nX", () -> next.getX());
        tab.addNumber("nY", () -> next.getY());
        tab.addNumber("nR", () -> next.getRotation().getDegrees());
        tab.addNumber("dX", () -> err.getX());
        tab.addNumber("dY", () -> err.getY());
        tab.addNumber("dR", () -> err.getRotation().getDegrees());
        tab.addNumber("Throttle", () -> throttle);
        tab.addNumber("Wheel", () -> wheel);
        tab.addBoolean("Finished", () -> finished);
    }

    private double clamp(double v, double min, double max) {
        return Math.min(Math.max(v, min), max);
    }

    private boolean atSetpoint() {
        return Math.abs(turnController.getPositionError()) <= 5.0 && Math.abs(driveController.getPositionError()) <= 2.5; 
    }

    @Override
    public void initialize() {
        params = poseStream.next();
        if (params.odomReset) {
            drivetrain.resetOdometry(new Pose2d());
        }
    }

    @Override
    public void execute() {
        if (poseStream.hasNext() && atSetpoint()) {
            params = poseStream.next();
            if (params.odomReset) {
                drivetrain.resetOdometry(new Pose2d());
            }
        }

        turnController.setI(Math.abs(turnController.getPositionError()) <= 10.0 ? .02 : 0.0);
        next = params.pose;
        err = next.relativeTo(drivetrain.getPose());

        if (err.getY() < 0) {
            err = new Pose2d(-err.getX(), err.getY(), err.getRotation());
        }

        driveController.setSetpoint(0);
        turnController.setSetpoint(next.getRotation().getDegrees() - clamp(err.getX() * params.turnRate, -MAX_TURN, MAX_TURN));

        double absErr = clamp(Math.abs(turnController.getPositionError()), 0.0, 90.0);
        throttle = driveController.calculate(-err.getY()) * (-absErr / 90.0 + 1);
        wheel = -turnController.calculate(drivetrain.getPose().getRotation().getDegrees());
        drivetrain.arcadeDrive(throttle, wheel);

        System.out.println(String.format("Drive: %f,%f,%f,%f", driveController.getSetpoint(), drivetrain.getPose().getY(), driveController.getPositionError(), throttle));
        System.out.println(String.format("Turn: %f,%f,%f,%f", turnController.getSetpoint(), drivetrain.getPose().getRotation().getDegrees(), turnController.getPositionError(), wheel));
    }

    @Override
    public void end(boolean interrupted) {
        finished = true;
        drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        return !poseStream.hasNext() && atSetpoint();
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(drivetrain);
    }
}
