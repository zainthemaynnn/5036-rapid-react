package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.ArmDown;
import frc.robot.commands.drive.DriveAuto;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class Taxi extends SequentialCommandGroup {
    public Taxi(Drivetrain drivetrain, Arm arm, Intake intake) {
        super(
            new WaitCommand(3.0),
            new ArmDown(arm),                                                                                                                                           
            new WaitCommand(1.0),
            new DriveAuto(drivetrain, +96.0)
        );
    }
}
