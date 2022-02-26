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
    private double degrees;
    private static final double INTEGRATOR_RANGE = 10.0;
    private static final double ki = 0.02;

    public TurnAuto(Drivetrain drivetrain, double degrees) {
        this.drivetrain = drivetrain;
        this.degrees = degrees;
        pid = new PIDController(0.007, 0, 1);
    }

    @Override
    public void initialize() {
        pid.reset();
        pid.setSetpoint(drivetrain.getHeading() + degrees);
    }

    @Override
    public void execute() {
        pid.setI(Math.abs(pid.getPositionError()) <= INTEGRATOR_RANGE ? ki : 0.0);
        double power = pid.calculate(drivetrain.getHeading());
        drivetrain.arcadeDrive(0.0, power);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        return pid.atSetpoint();
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(drivetrain);
    }
}
