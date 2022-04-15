// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.Set;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Drivetrain;

public class DriveAutoGood implements Command {
  private Drivetrain drivetrain;
  private PIDController velocity, position;
  private double distance;

  /** Creates a new DriveAutoGood. */
  public DriveAutoGood(Drivetrain drivetrain, double distance) {
    this.drivetrain = drivetrain;
    this.distance = distance;
    velocity = new PIDController(1.5e-2, 1e-2, 3e-4);
    position = new PIDController(2, 0, 0);
    position.setTolerance(1.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    position.setSetpoint(drivetrain.getEncPosition() + distance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.arcadeDrive(
      velocity.calculate(
        drivetrain.getEncVelocity(),
        position.calculate(
          drivetrain.getEncPosition()
        )
      ),
      0.0
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    velocity.reset();
    position.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return position.atSetpoint();
  }

  @Override
  public Set<Subsystem> getRequirements() {
    return Set.of(drivetrain);
  }
}
