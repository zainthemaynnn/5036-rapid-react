package frc.robot.commands.drive;

import java.util.Set;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.math.Limits;
import frc.math.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;

public class TurnAutoJank implements Command {
    private Drivetrain drivetrain;
    private PIDController pid;
    private double degrees;
    private int count;

    public TurnAutoJank(Drivetrain drivetrain, double degrees, boolean jankyMode) {
        this.drivetrain = drivetrain;
        this.degrees = degrees;
        pid = new PIDController(4e-3, 1.05e-3, 0);
        if (jankyMode) {
            pid.setTolerance(10.0);
        } else {
            pid.setTolerance(2.0);
        }
    }

    public TurnAutoJank(Drivetrain drivetrain, double degrees) {
        this(drivetrain, degrees, false);
    }

    @Override
    public void initialize() {
        System.out.println(drivetrain.getAngle());
        pid.reset();
        System.out.println(degrees);
        pid.setSetpoint(degrees);
        count = 0;
    }

    @Override
    public void execute() {
        pid.setI(pid.getPositionError() < 10.0 ? 1e-3 : 0);
        SmartDashboard.putNumber("err", pid.getPositionError());
        double power = Limits.siglim(pid.calculate(drivetrain.getAngle()), 0.12, 1);
        drivetrain.arcadeDrive(0.0, power);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println(drivetrain.getAngle());
        drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        count = pid.atSetpoint() ? count + 1 : 0;
        return count >= 5;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(drivetrain);
    }
}
