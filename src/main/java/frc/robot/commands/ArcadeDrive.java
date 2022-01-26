package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.subsystems.Drivetrain;
import frc.robot.Gamepad;

public class ArcadeDrive implements Command {
    private Drivetrain drivetrain;
    private Gamepad driver;

    public ArcadeDrive(Gamepad driver, Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.driver = driver;
    }

    public void execute() {
        drivetrain.arcadeDrive(
            driver.getAxis(Gamepad.Axis.LEFT_Y, .02),
            driver.getAxis(Gamepad.Axis.RIGHT_X, .02)
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
