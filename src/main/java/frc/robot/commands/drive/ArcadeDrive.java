package frc.robot.commands.drive;

import java.util.Set;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.subsystems.Drivetrain;

public class ArcadeDrive implements Command {
    private Drivetrain drivetrain;
    private DoubleSupplier throttle, wheel;

    public ArcadeDrive(Drivetrain drivetrain, DoubleSupplier throttle, DoubleSupplier wheel) {
        this.drivetrain = drivetrain;
        this.throttle = throttle;
        this.wheel = wheel;
    }

    public void execute() {
        drivetrain.arcadeDrive(
            throttle.getAsDouble(),
            wheel.getAsDouble()
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
