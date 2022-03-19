// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.FileSystem;
import java.util.Iterator;
import java.util.List;
import java.util.Set;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.devtools.PIDTuner;
import frc.robot.subsystems.Drivetrain;


public class DriveToPoint implements Command {
  private static Drivetrain drivetrain;
  private static final double Y_ERR = 1.0;
  private static final double R_ERR = 5.0;
  private PIDController driveController, turnController;
  private Pose2d target = new Pose2d();
  private static Pose2d err = new Pose2d();
  private static ShuffleboardTab tab = Shuffleboard.getTab("DriveToPoint");
  private double turnRate, maxSpeed;

  public static void init(Drivetrain drivetrain) {
    DriveToPoint.drivetrain = drivetrain;
    tab.addNumber("X", () -> drivetrain.getPose().getX());
    tab.addNumber("Y", () -> drivetrain.getPose().getY());
    tab.addNumber("R", () -> drivetrain.getPose().getRotation().getDegrees());
    tab.addNumber("dX", () -> err.getX());
    tab.addNumber("dY", () -> err.getY());
    tab.addNumber("dR", () -> err.getRotation().getDegrees());
  }

  public DriveToPoint(Drivetrain drivetrain, Pose2d target, double turnRate, double maxSpeed) {
    this.target = target;
    this.turnRate = turnRate;
    this.maxSpeed = maxSpeed;
    driveController = new PIDController(0.015, 0, 0.03);
    turnController = new PIDController(0.008, /*0.0*/0.02, 10);
  }

  private double clamp(double x, double min, double max) {
    return Math.min(Math.max(x, min), max);
  }

  @Override
  public void initialize() {
    driveController.reset();
    turnController.reset();
  }

  @Override
  public void execute() {
    err = target.relativeTo(drivetrain.getPose());

    if (err.getY() < 0) {
      err = new Pose2d(-err.getX(), err.getY(), err.getRotation());
    }

    driveController.setSetpoint(err.getY());
    turnController.setSetpoint(err.getRotation().getDegrees() - clamp(err.getX() * turnRate, -90.0, 90.0));
    System.out.println(turnController.getSetpoint());
    double absErr = clamp(Math.abs(turnController.getPositionError()), 0.0, 90.0);
    double throttle = clamp(driveController.calculate(0.0) * (-absErr / 90.0 + 1), 0, maxSpeed);
    double wheel = -turnController.calculate(0.0);
    drivetrain.arcadeDrive(throttle, wheel);
  }

  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      System.out.println("success");
    }
    drivetrain.stop();
  }

  @Override
  public boolean isFinished() {
    return Math.abs(err.getY()) < Y_ERR;
  }

  @Override
  public Set<Subsystem> getRequirements() {
    return Set.of(drivetrain);
  }
}
