package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.arm.ArmDown;
import frc.robot.commands.intake.EjectCargo;
import frc.robot.commands.intake.StopIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class Snipe extends SequentialCommandGroup {
    public Snipe(Drivetrain drivetrain, Arm arm, Intake intake) {
        super(
            new ArmDown(arm),
            new WaitUntilCommand(arm::isDown),
            new EjectCargo(intake),
            new WaitCommand(0.5),
            new StopIntake(intake)
        );
    }
}
