package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

public class StopIntake extends InstantCommand {
    public StopIntake(Intake intake) {
        super(
            () -> intake.runPercent(0),
            intake
        );
    }
}
