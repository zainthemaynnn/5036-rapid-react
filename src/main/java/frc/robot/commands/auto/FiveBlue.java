// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveToPoint;
import frc.robot.subsystems.Drivetrain;

public class FiveBlue extends SequentialCommandGroup {
  public FiveBlue(Drivetrain drivetrain) {
    super(
      new TwoBlue(drivetrain)
    );
  }
}
