package frc.robot.commands.arm;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Arm;

public class LowerArm implements Command {
    private Arm arm;

    public LowerArm(Arm arm) {
        this.arm = arm;
    }
    
    public void execute() {

    }

    public Set<Subsystem> getRequirements() {
        return Set.of(arm);
    }
}
