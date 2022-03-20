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
    private double count;

    public DriveAuto(Drivetrain drivetrain, double distance) {
        this.drivetrain = drivetrain;
        this.distance = distance;
        driveController = new PIDController(0.012, 0.0001, 0);//new PIDController(0.015, 0, 1);
        driveController.setTolerance(1.0);
        turnController = new PIDController(0.0072, 0.01, 0.06);
    }

    @Override
    public void initialize() {
        driveController.setIntegratorRange(0.00005, 0.00015);
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
        SmartDashboard.putNumber("e", driveController.getPositionError());
        SmartDashboard.putNumber("et", turnController.getPositionError());
        count = driveController.atSetpoint() ? count + 1 : 0;
        return count >= 10;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(drivetrain);
    }
}
