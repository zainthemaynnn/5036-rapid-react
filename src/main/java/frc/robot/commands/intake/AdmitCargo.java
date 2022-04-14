package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

public class AdmitCargo extends InstantCommand {
    public AdmitCargo(Intake intake) {
        super(
            () -> intake.runPercent(-.7),
            intake
        );
    }
}
