package frc.robot.commands.drive;

import java.util.Set;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;

public class DriveAuto implements Command {
    private Drivetrain drivetrain;
    private PIDController driveController, turnController;
    private double distance;
    private double maxSpeed;

    public DriveAuto(Drivetrain drivetrain, double distance, double maxSpeed, boolean stopAtWall) {
        this.drivetrain = drivetrain;
        this.distance = distance;
        driveController = new PIDController(1.2e-2, 1e-4, 0);
        driveController.setTolerance(stopAtWall ? 10.0 : 2.0, stopAtWall ? 0.10 : 1);
        turnController = new PIDController(7e-30, 0, 8e-4);
    }

    public DriveAuto(Drivetrain drivetrain, double distance, double maxSpeed) {
        this(drivetrain, distance, maxSpeed, false);
    }

    public DriveAuto(Drivetrain drivetrain, double distance, boolean stopAtWall) {
        this(drivetrain, distance, 1, stopAtWall);
    }

    public DriveAuto(Drivetrain drivetrain, double distance) {
        this(drivetrain, distance, false);
    }

    @Override
    public void initialize() {
        driveController.reset();
        driveController.setSetpoint(drivetrain.getEncPosition() + distance);
        turnController.reset();
        turnController.setSetpoint(drivetrain.getAngle());
    }

    @Override
    public void execute() {
        drivetrain.arcadeDrive(
            driveController.calculate(drivetrain.getEncPosition()),
            turnController.calculate(drivetrain.getAngle())
        );
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        SmartDashboard.putNumber("e", driveController.getPositionError());
        SmartDashboard.putNumber("et", turnController.getPositionError());
        return driveController.atSetpoint();
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(drivetrain);
    }
}
