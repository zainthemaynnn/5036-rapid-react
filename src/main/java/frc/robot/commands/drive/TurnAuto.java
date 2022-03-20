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
    private int count;
    private boolean jankyMode;

    public TurnAuto(Drivetrain drivetrain, double degrees, boolean jankyMode) {
        this.drivetrain = drivetrain;
        this.degrees = degrees;
        this.jankyMode = jankyMode;
        pid = new PIDController(0.0072, 0.001, 0.06);//new PIDController(0.008, 0.00001, 0.05);
    }

    public TurnAuto(Drivetrain drivetrain, double degrees) {
        this(drivetrain, degrees, false);
    }

    @Override
    public void initialize() {
        //pid.setIntegratorRange(SmartDashboard.getNumber("lbt", 0), SmartDashboard.getNumber("rbt", 0));//pid.setIntegratorRange(0.0025, 0.004);
        System.out.println(drivetrain.getHeading());
        pid.reset();
        System.out.println(degrees);
        pid.setSetpoint(degrees);
        pid.setTolerance(jankyMode ? 4.0 : 1.5);
        count = 0;
    }

    private double clamp(double x, double min, double max) {
        return Math.max(Math.min(x, max), min);
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("err", pid.getPositionError());
        double power = clamp(pid.calculate(drivetrain.getHeading()), -.5, .5);
        drivetrain.arcadeDrive(0.0, power);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println(drivetrain.getHeading());
        drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        count = pid.atSetpoint() ? count + 1 : 0;
        return count >= 10;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(drivetrain);
    }
}
