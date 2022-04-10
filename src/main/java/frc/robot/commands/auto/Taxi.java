package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.DriveAuto;
import frc.robot.commands.drive.TurnAutoJank;
import frc.robot.commands.intake.AdmitCargo;
import frc.robot.commands.intake.EjectCargo;
import frc.robot.commands.intake.StopIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class Taxi extends SequentialCommandGroup {
    public Taxi(Drivetrain drivetrain, Arm arm, Intake intake) {
        super(
            new InstantCommand(() -> System.out.println("begin")),
            new InstantCommand(() -> drivetrain.resetOdometry(new Pose2d())),
            new InstantCommand(() -> drivetrain.setRampRate(0.65)),

            new WaitCommand(3.0),
            new InstantCommand(() -> arm.override(true)),
            new WaitCommand(1.0),
            new DriveAuto(drivetrain, +96.0),

            new InstantCommand(() -> drivetrain.setRampRate(0)),
            new InstantCommand(() -> System.out.println("complete"))
        );
    }
}
