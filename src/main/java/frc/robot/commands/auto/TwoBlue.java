// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveToPoint;
import frc.robot.subsystems.Drivetrain;

public class TwoBlue extends SequentialCommandGroup {
  public TwoBlue(Drivetrain drivetrain) {
    super(
      new DriveToPoint(drivetrain, new Pose2d(-18.0, 31.0, new Rotation2d(Math.toRadians(0))), 1.5)//,
      //new DriveToPoint(drivetrain, new Pose2d(17.90, 60.92, new Rotation2d(-90)), 1.0)
    );
  }
}
