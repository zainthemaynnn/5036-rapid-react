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
        // /driveController.setTolerance(stopAtWall ? 10.0 : 3.0, stopAtWall ? 0.10 : 1);
        turnController = new PIDController(7e-3, /*1e-2*/0, 8e-4);
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
        driveController.setPID(SmartDashboard.getNumber("P", 0), Math.abs(driveController.getPositionError()) < 10.0 ? SmartDashboard.getNumber("I", 0) : 0, SmartDashboard.getNumber("D", 0));
        //driveController.setIntegratorRange(0.00005, 0.00015);
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
        return driveController.atSetpoint();
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(drivetrain);
    }
}
