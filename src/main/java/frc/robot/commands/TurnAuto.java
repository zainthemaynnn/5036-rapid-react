package frc.robot.commands;

import java.util.Set;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.math.controller.PIDController;
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
        target = drivetrain.gyro.getAngle() + degrees;
    }

    public void execute() {
        double power = pid.calculate(drivetrain.gyro.getAngle(), target);
        drivetrain.motorL.set(power);
        drivetrain.motorR.set(-power);
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
