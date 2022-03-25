package frc.robot.commands.drive;

import java.util.Set;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;

public class DriveAutoJank implements Command {
    private Drivetrain drivetrain;
    private PIDController driveController, turnController;
    private double distance;
    private double maxSpeed;

    public DriveAutoJank(Drivetrain drivetrain, double distance, double maxSpeed, boolean stopAtWall) {
        this.drivetrain = drivetrain;
        this.distance = distance;
        driveController = new PIDController(1.5e-2, 9e-2, 6.5e-3);
        driveController.setTolerance(stopAtWall ? 10.0 : 2.0, 0.10);
        turnController = new PIDController(3e-2, /*1e-2*/0, 8e-4);
    }

    public DriveAutoJank(Drivetrain drivetrain, double distance, double maxSpeed) {
        this(drivetrain, distance, maxSpeed, false);
    }

    public DriveAutoJank(Drivetrain drivetrain, double distance, boolean stopAtWall) {
        this(drivetrain, distance, 1, stopAtWall);
    }

    public DriveAutoJank(Drivetrain drivetrain, double distance) {
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
        double pow = driveController.calculate(drivetrain.getEncPosition());
        drivetrain.arcadeDrive(
            pow,
            turnController.getSetpoint() - drivetrain.getAngle() > 0 ? -0.15 : 0.15
        );
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        SmartDashboard.putNumber("e", driveController.getPositionError());
        SmartDashboard.putNumber("et", turnController.getSetpoint() - drivetrain.getAngle());
        return driveController.atSetpoint();
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(drivetrain);
    }
}
