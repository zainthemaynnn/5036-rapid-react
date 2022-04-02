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

public class ThreeBlue extends SequentialCommandGroup {
  public ThreeBlue(Drivetrain drivetrain, Arm arm, Intake intake) {
    super(
      new InstantCommand(() -> System.out.println("begin")),
      new InstantCommand(() -> drivetrain.resetOdometry(new Pose2d())),
      new InstantCommand(() -> drivetrain.setRampRate(0.65)),
      new AdmitCargo(intake),
      new InstantCommand(() -> arm.override(true)),

      new WaitUntilCommand(arm::isDown),

      new InstantCommand(() -> System.out.println("1")),
      //new DriveToPoint(drivetrain, new Pose2d(-12, +70.0, new Rotation2d(Math.toRadians(0))), 0.55, 0.70),
      new DriveAuto(drivetrain, +72.0),
      new WaitCommand(0.15),
      new StopIntake(intake),

      new InstantCommand(() -> System.out.println("2")),
      new InstantCommand(() -> System.out.println(drivetrain.getAngle())),
      new TurnAuto(drivetrain, +94.5),
      new AdmitCargo(intake),
      new WaitCommand(0.1),

      new InstantCommand(() -> System.out.println("3")),
      new DriveAuto(drivetrain, +89.0),
      new WaitCommand(0.1),

      new DriveAuto(drivetrain, -78.0),
      new StopIntake(intake),
      new WaitCommand(0.1),

      new InstantCommand(() -> System.out.println("4")),
      new TurnAuto(drivetrain, +186.0),

      new InstantCommand(() -> arm.override(false)),
      new WaitUntilCommand(arm::isUp),
      new WaitCommand(0.1),

      new DriveAuto(drivetrain, +70.0, true),

      new InstantCommand(() -> System.out.println("5")),
      new WaitCommand(0.1),

      new EjectCargo(intake),
      new WaitCommand(1.0),
      new StopIntake(intake),

      new DriveAuto(drivetrain, -30.0),
      new TurnAuto(drivetrain, +270.0),

      new InstantCommand(() -> drivetrain.setRampRate(0)),
      new InstantCommand(() -> System.out.println("complete"))
    );
  }
}
