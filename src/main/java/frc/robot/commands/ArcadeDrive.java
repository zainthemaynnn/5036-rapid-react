package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.subsystems.Drivetrain;
import frc.robot.Gamepad;

public class ArcadeDrive implements Command {
    private Drivetrain drivetrain;
    private Gamepad driver;

    public ArcadeDrive(Drivetrain drivetrain, Gamepad driver) {
        this.drivetrain = drivetrain;
        this.driver = driver;
    }

    public void execute() {
        double throttle = driver.getAxisWithDeadband(Gamepad.Axis.LeftY, .02);
        double wheel = driver.getAxisWithDeadband(Gamepad.Axis.RightX, .02);
        drivetrain.motorL.set(throttle + wheel);
        drivetrain.motorR.set(throttle - wheel);
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
