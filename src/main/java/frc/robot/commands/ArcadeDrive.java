package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.subsystems.Drivetrain;
import frc.robot.Controller;

public class ArcadeDrive implements Command {
    private Drivetrain drivetrain;
    private Controller driver;

    public ArcadeDrive(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    public void execute() {
        double y = driver.getAxis(Controller.Axis.LeftY);
        double x = driver.getAxis(Controller.Axis.RightX);
        drivetrain.motorL.set(y + x);
        drivetrain.motorR.set(y - x);
    }

    public void end() {
        drivetrain.stop();
    }

    public boolean isFinished() {
        return false;
    }

    public Set<Subsystem> getRequirements() {
        return Set.of(drivetrain);
    }
}
