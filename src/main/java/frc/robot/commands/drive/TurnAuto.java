package frc.robot.commands.drive;

import java.util.Set;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.math.Limits;
import frc.math.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;

public class TurnAuto implements Command {
    private Drivetrain drivetrain;
    private PIDController pid;
    private double degrees;
    private int count;

    public TurnAuto(Drivetrain drivetrain, double degrees) {
        this.drivetrain = drivetrain;
        this.degrees = degrees;
        pid = new PIDController(7e-3, 1e-2, 8e-4);
        pid.setTolerance(2.0);
    }

    @Override
    public void initialize() {
        //pid.setIntegratorRange(SmartDashboard.getNumber("lbt", 0), SmartDashboard.getNumber("rbt", 0));//pid.setIntegratorRange(0.0025, 0.004);
        System.out.println(drivetrain.getAngle());
        pid.reset();
        System.out.println(degrees);
        pid.setSetpoint(degrees);
        count = 0;
    }

    @Override
    public void execute() {
        pid.setI(pid.getPositionError() < 10.0 ? 1e-2 : 0);
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
        return count >= 10;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(drivetrain);
    }
}
