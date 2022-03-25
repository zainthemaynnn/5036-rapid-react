package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.intake.AdmitCargo;
import frc.robot.commands.intake.EjectCargo;
import frc.robot.commands.intake.StopIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class Snipe extends ParallelCommandGroup {
    public Snipe(Drivetrain drivetrain, Arm arm, Intake intake) {
        super(
            new InstantCommand(() -> System.out.println("begin")),
            new InstantCommand(() -> drivetrain.resetOdometry(new Pose2d())),
            new InstantCommand(() -> drivetrain.setRampRate(0.65)),

            new InstantCommand(() -> arm.override(true)),
            new WaitUntilCommand(arm::isDown),
            new EjectCargo(intake),
            new WaitCommand(0.5),
            new StopIntake(intake),

            new InstantCommand(() -> drivetrain.setRampRate(0)),
            new InstantCommand(() -> System.out.println("complete"))
        );
    }
}
