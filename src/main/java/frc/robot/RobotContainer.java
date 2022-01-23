// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.CurvatureDrive;
import frc.robot.commands.DriveAuto;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.TurnAuto;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  // Subsystems
  private final LedSubsystem m_ledSubsystem = new LedSubsystem();
  private final Drivetrain m_drivetrain = new Drivetrain(motorL, motorR, encoderL, encoderR, gyro);
  private final Limelight m_limelight = new Limelight();

  // Commands  
  
  private final CurvatureDrive m_curvatureDrive = new CurvatureDrive(driver, m_drivetrain);
  
  // Auto Commands

  private final TurnAuto m_turnAuto = new TurnAuto(m_drivetrain, degrees);
  private final DriveAuto m_driveAuto = new DriveAuto(m_drivetrain, distance, reverse);

  // Buttons
  private final RunCommand m_stopShooter = new RunCommand(() -> m_shooter.stopShooters(), m_shooter);
  private final RunCommand m_indexerReverse = new RunCommand(()->m_indexer.runIndexer(-0.6,-0.6,-0.6), m_indexer);
  private final RunCommand m_indexerStop = new RunCommand(()->m_indexer.runIndexer(0,0,0), m_indexer);
  private final RunCommand m_intakeOut= new RunCommand(() -> m_intake.intakeOut(), m_intake);
  private final RunCommand m_intakeIn= new RunCommand(() -> m_intake.intakeIn(), m_intake);



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_driveAuto;
  }
}
