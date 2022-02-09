package frc.robot.commands.drive;

import java.util.Set;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.Drivetrain;

public class TurnAuto implements Command {
    private Drivetrain drivetrain;
    private PIDController pid;
    private static final double kp = 0.0125, ki = 0, kd = 2;
    private double target;
    private double degrees;

    public TurnAuto(Drivetrain drivetrain, double degrees) {
        this.drivetrain = drivetrain;
        this.degrees = degrees;
        pid = new PIDController(kp, ki, kd);
    }

    public void initialize() {
        target = drivetrain.getHeading() + degrees;
        pid.reset();
    }

    public void execute() {
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
