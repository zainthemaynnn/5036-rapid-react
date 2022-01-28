package frc.robot.commands.intake;

import java.util.Set;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Intake;

public class AdmitCargo implements Command {
    private Intake intake;
    private BooleanSupplier active;

    public AdmitCargo(Intake intake) {
        this.intake = intake;
    }

    public void initialize() {
        // TODO: lower arm
    }

    public void execute() {
        intake.run();
    }

    public void end() {
        // TODO: raise arm
    }

    public Set<Subsystem> getRequirements() {
        return Set.of(intake);
    }
}
