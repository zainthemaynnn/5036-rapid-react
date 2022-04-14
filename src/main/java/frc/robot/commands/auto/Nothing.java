package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class Nothing extends SequentialCommandGroup {
    public Nothing(Drivetrain drivetrain, Arm arm, Intake intake) {
        super();
    }
}
