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
    private static final double ki = 0;

    public TurnAuto(Drivetrain drivetrain, double degrees) {
        this.drivetrain = drivetrain;
        this.degrees = degrees;
        pid = new PIDController(0.007, 0.02, 1);
    }

    public void initialize() {
        pid.reset();
        pid.setSetpoint(drivetrain.getHeading() + degrees);
    }

    public void execute() {
        SmartDashboard.putNumber("setpoint", pid.getSetpoint());
        SmartDashboard.putNumber("err", pid.getPositionError());
        pid.setP(SmartDashboard.getNumber("P", 0.0));
        pid.setI(Math.abs(pid.getPositionError()) <= INTEGRATOR_RANGE ? SmartDashboard.getNumber("I", 0.0) : 0.0);
        pid.setD(SmartDashboard.getNumber("D", 0.0));
        double power = pid.calculate(drivetrain.getHeading());
        SmartDashboard.putNumber("power", power);
        drivetrain.arcadeDrive(0.0, power);
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
