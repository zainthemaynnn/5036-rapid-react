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
        turnController = new PIDController(0.008, 0, 10);
    }

    @Override
    public void initialize() {
        driveController.reset();
        driveController.setSetpoint(drivetrain.getEncAvg() - distance);
        turnController.reset();
        turnController.setSetpoint(drivetrain.getHeading());
    }

    @Override
    public void execute() {
        driveController.setPID(SmartDashboard.getNumber("P", 0), SmartDashboard.getNumber("I", 0), SmartDashboard.getNumber("D", 0));
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
        return driveController.atSetpoint() && turnController.atSetpoint();
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(drivetrain);
    }
}
