package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.Intake;

public class EjectCargo extends InstantCommand {
    public EjectCargo(Intake intake) {
        super(
            () -> intake.runPercent(-1),
            intake
        );
    }
}
