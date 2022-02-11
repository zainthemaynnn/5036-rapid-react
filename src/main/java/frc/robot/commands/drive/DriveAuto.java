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
    private static int kp, ki, kd;
    private double distance;

    public DriveAuto(Drivetrain drivetrain, double distance) {
        this.drivetrain = drivetrain;
        this.distance = distance;
        kp = ki = kd = 0; // TODO
        driveController = new PIDController(kp, ki, kd);
        turnController = new PIDController(0, 0, 0);
    }

    public void initialize() {
        driveController.reset();
        driveController.setSetpoint(drivetrain.getAvgEncDistance() - distance);
        turnController.reset();
        turnController.setSetpoint(drivetrain.getHeading());
    }

    public void execute() {
        driveController.setPID(SmartDashboard.getNumber("P", 0), SmartDashboard.getNumber("I", 0), SmartDashboard.getNumber("D", 0));
        drivetrain.arcadeDrive(
            driveController.calculate(drivetrain.getAvgEncDistance()),
            turnController.calculate(drivetrain.getHeading())
        );
    }

    public void end() {
        drivetrain.stop();
    }

    public boolean isFinished() {
        return driveController.atSetpoint() && turnController.atSetpoint();
    }

    public Set<Subsystem> getRequirements() {
        return Set.of(drivetrain);
    }
}
