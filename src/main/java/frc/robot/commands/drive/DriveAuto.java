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

    public DriveAuto(Drivetrain drivetrain, double distance) {
        this.drivetrain = drivetrain;
        this.distance = distance;
        driveController = new PIDController(0.015, 0, 0.03);
        driveController.setTolerance(1.5, .10);
        turnController = new PIDController(0.007, 0, 10);
        turnController.setTolerance(2.0);
    }

    @Override
    public void initialize() {
        driveController.reset();
        driveController.setSetpoint(drivetrain.getEncAvg() + distance);
        turnController.reset();
        turnController.setSetpoint(drivetrain.getHeading());
    }

    @Override
    public void execute() {
        drivetrain.arcadeDrive(
            driveController.calculate(drivetrain.getEncAvg()),
            turnController.calculate(drivetrain.getHeading())
        );
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        return driveController.atSetpoint();
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(drivetrain);
    }
}
