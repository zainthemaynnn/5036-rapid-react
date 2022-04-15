package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.ArmDown;
import frc.robot.commands.arm.ArmUp;
import frc.robot.commands.drive.DriveAuto;
import frc.robot.commands.drive.TurnAutoJank;
import frc.robot.commands.intake.AdmitCargo;
import frc.robot.commands.intake.EjectCargo;
import frc.robot.commands.intake.StopIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class TwoBlue extends SequentialCommandGroup {
    public TwoBlue(Drivetrain drivetrain, Arm arm, Intake intake) {
        super(
            new AdmitCargo(intake),
            new ArmDown(arm),

            new DriveAuto(drivetrain, +72.0),
            new StopIntake(intake),

            new ArmUp(arm)
                .alongWith(new AdmitCargo(intake)),

            new DriveAuto(drivetrain, -98.0),
            new StopIntake(intake),
            new TurnAutoJank(drivetrain, +162.0),

            new EjectCargo(intake),
            new WaitCommand(0.5),
            new StopIntake(intake)
        );
    }
}
