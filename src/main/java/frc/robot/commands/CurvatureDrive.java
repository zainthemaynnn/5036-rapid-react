package frc.robot.commands;

import java.util.Set;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Gamepad;
import frc.robot.Constants;

public class CurvatureDrive implements Command {
    public Gamepad driver;
    public Drivetrain drivetrain;

    public CurvatureDrive(Gamepad driver, Drivetrain drivetrain) {
        this.driver = driver;
        this.drivetrain = drivetrain;
    }

    public void execute() {
        drivetrain.curvatureDrive(
            driver.getAxisWithDeadband(Gamepad.Axis.LEFT_Y, .02),
            driver.getAxisWithDeadband(Gamepad.Axis.RIGHT_X, .02)
        );
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
