package frc.robot.commands.drive;

import java.util.Iterator;
import java.util.List;
import java.util.Set;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        driveController.setSetpoint(drivetrain.getPose().getX());
        turnController.setSetpoint(drivetrain.getPose().getY());

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

        next = new Pose2d(0, 60.92, new Rotation2d());//poseStream.next();
        Transform2d t = createTransform(drivetrain.getPose(), next);
        diff = new Pose2d(t.getTranslation(), t.getRotation());
        driveController.setSetpoint(diff.getY());
        turnController.setSetpoint(next.getRotation().getDegrees() - clamp(diff.getX() * turnRate, -MAX_TURN, MAX_TURN));
    }

    private double clamp(double v, double min, double max) {
        return Math.min(Math.max(v, min), max);
    }

    private boolean atSetpoint() {
        return driveController.atSetpoint() && turnController.atSetpoint();
    }

    private Transform2d createTransform(Pose2d initial, Pose2d last) {
        Translation2d m_translation =
        last.getTranslation()
            .minus(initial.getTranslation())
            .rotateBy(initial.getRotation().unaryMinus());

        Rotation2d m_rotation = last.getRotation().minus(initial.getRotation());
        return new Transform2d(m_translation, m_rotation);
    }

    @Override
    public void initialize() {
        //drivetrain.resetOdometry(new Pose2d());
        /*next = new Pose2d(17.90, 60.92, new Rotation2d());//poseStream.next();
        Transform2d t = createTransform(drivetrain.getPose(), next);
        diff = new Pose2d(t.getTranslation(), t.getRotation());
        driveController.setSetpoint(diff.getY());
        turnController.setSetpoint(next.getRotation().getDegrees() - clamp(diff.getX() * turnRate, -MAX_TURN, MAX_TURN));*/
    }

    @Override
    public void execute() {
        /*if (poseStream.hasNext() && atSetpoint()) {
            next = new Pose2d(0, 60.92, new Rotation2d(Math.toRadians(90)));//poseStream.next();
            diff = next.relativeTo(drivetrain.getPose());
            driveController.setSetpoint(diff.getY());
            turnController.setSetpoint(next.getRotation().getDegrees() - clamp(diff.getX() * turnRate, -MAX_TURN, MAX_TURN));
        }*/

        double absErr = clamp(Math.abs(turnController.getPositionError()), 0.0, 90.0);
        throttle = driveController.calculate(drivetrain.getEncAvg()) * (1 - absErr / 90.0);
        wheel = turnController.calculate(drivetrain.getHeading());
        SmartDashboard.putNumber("clamped", clamp(wheel, -.1, .1));

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
