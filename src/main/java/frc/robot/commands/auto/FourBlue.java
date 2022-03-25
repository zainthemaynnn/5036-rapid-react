// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.arm.ArmDown;
import frc.robot.commands.arm.WaitForArmDown;
import frc.robot.commands.drive.DriveAuto;
import frc.robot.commands.drive.DriveAutoJank;
import frc.robot.commands.drive.DriveToPoint;
import frc.robot.commands.drive.TurnAuto;
import frc.robot.commands.drive.TurnAutoJank;
import frc.robot.commands.drive.TurnAutoCreative;
import frc.robot.commands.intake.AdmitCargo;
import frc.robot.commands.intake.EjectCargo;
import frc.robot.commands.intake.StopIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class FourBlue extends SequentialCommandGroup {
  private double distanceTraveled = 0.0;

  public FourBlue(Drivetrain drivetrain, Arm arm, Intake intake) {
    super(
      new InstantCommand(() -> System.out.println("begin")),
      new InstantCommand(() -> drivetrain.resetOdometry(new Pose2d())),
      new InstantCommand(() -> drivetrain.setRampRate(0.65)),
      new AdmitCargo(intake),
      new InstantCommand(() -> arm.override(true)),

      new DriveAuto(drivetrain, +36.0),
      new StopIntake(intake),

      new InstantCommand(() -> {
        arm.override(false);
        intake.runPercent(.70);
      }),

      new DriveAuto(drivetrain, -62.0),
      new StopIntake(intake),
      new TurnAutoJank(drivetrain, +162.0),

      new EjectCargo(intake),
      new WaitCommand(0.5),
      new StopIntake(intake),

      new TurnAutoJank(drivetrain, +284.5),

      new InstantCommand(() -> arm.override(true)),
      new WaitUntilCommand(arm::isDown),
      new EjectCargo(intake),

      new WaitCommand(0.1),

      new ParallelCommandGroup(
        new DriveAutoJank(drivetrain, +178.0, 0.20, true),
        new SequentialCommandGroup(
          new WaitUntilCommand(() -> drivetrain.getEncAvg() >= +120.0),
          new AdmitCargo(intake)
        )
      ),

      new WaitCommand(1.0),

      new StopIntake(intake),

      new DriveAutoJank(drivetrain, -155.0),

      new WaitCommand(1.0),

      new TurnAutoCreative(drivetrain, +155.0),

      new InstantCommand(() -> arm.override(false)),
      new DriveAuto(drivetrain, +40.0, true),
      new EjectCargo(intake),
      new WaitCommand(1.0),
      new StopIntake(intake),

      new InstantCommand(() -> System.out.println("complete"))
    );
  }
}
