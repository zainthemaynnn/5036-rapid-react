// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.arm.ArmDown;
import frc.robot.commands.arm.ArmUp;
import frc.robot.commands.drive.DriveAuto;
import frc.robot.commands.drive.TurnAuto;
import frc.robot.commands.intake.AdmitCargo;
import frc.robot.commands.intake.EjectCargo;
import frc.robot.commands.intake.StopIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class ThreeBlue extends SequentialCommandGroup {
  public ThreeBlue(Drivetrain drivetrain, Arm arm, Intake intake) {
    super(
      new AdmitCargo(intake),
      new ArmDown(arm),

      new WaitUntilCommand(arm::isDown),

      //new DriveToPoint(drivetrain, new Pose2d(-12, +70.0, new Rotation2d(Math.toRadians(0))), 0.55, 0.70),
      new DriveAuto(drivetrain, +72.0),
      new WaitCommand(0.15),
      new StopIntake(intake),

      new TurnAuto(drivetrain, +94.5),
      new AdmitCargo(intake),
      new WaitCommand(0.1),

      new DriveAuto(drivetrain, +99.0),
      new WaitCommand(0.1),

      new DriveAuto(drivetrain, -88.0),
      new StopIntake(intake),
      new WaitCommand(0.1),

      new TurnAuto(drivetrain, +186.0),

      new ArmUp(arm),
      new WaitUntilCommand(arm::isUp),
      new WaitCommand(0.1),

      new DriveAuto(drivetrain, +70.0, true),

      new WaitCommand(0.1),

      new EjectCargo(intake),
      new WaitCommand(1.0),
      new StopIntake(intake),

      new DriveAuto(drivetrain, -30.0),
      new TurnAuto(drivetrain, +270.0)
    );
  }
}
