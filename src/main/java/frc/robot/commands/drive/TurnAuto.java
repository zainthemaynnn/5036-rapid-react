package frc.robot.commands.drive;

import java.util.Set;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;

public class TurnAuto implements Command {
    private Drivetrain drivetrain;
    private PIDController pid;
    private static int kp, ki, kd;
    private double target;

    public TurnAuto(Drivetrain drivetrain, double degrees) {
        this.drivetrain = drivetrain;
        kp = ki = kd = 0; // TODO
        pid = new PIDController(kp, ki, kd);
        target = drivetrain.getHeading() + degrees;
        SmartDashboard.putNumber("P", 0);
        SmartDashboard.putNumber("I", 0);
        SmartDashboard.putNumber("D", 0);
    }

    public void execute() {
        pid.setPID(SmartDashboard.getNumber("P", 0), SmartDashboard.getNumber("I", 0), SmartDashboard.getNumber("D", 0));
        SmartDashboard.putNumber("heading", drivetrain.getHeading());
        SmartDashboard.putNumber("target", target);
        drivetrain.arcadeDrive(0.0, pid.calculate(drivetrain.getHeading(), target));
    }

    public void end() {
        drivetrain.stop();
    }

    public boolean isFinished() {
        return pid.atSetpoint();
    }

    public Set<Subsystem> getRequirements() {
        return Set.of(drivetrain);
    }
}
