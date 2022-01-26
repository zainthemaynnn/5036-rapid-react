// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.SPI;
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

  PowerDistribution pdp = new PowerDistribution();

  private final Limelight m_limelight = new Limelight();
  private final LedSubsystem m_ledSubsystem = new LedSubsystem();

  private final Drivetrain m_drivetrain = new Drivetrain(
    new MotorControllerGroup(
      new CANSparkMax(RobotMap.CAN.FRONT_MOTOR_LEFT.id(), MotorType.kBrushless),
      new CANSparkMax(RobotMap.CAN.BACK_MOTOR_LEFT.id(), MotorType.kBrushless)
    ),
    new MotorControllerGroup(
      new CANSparkMax(RobotMap.CAN.FRONT_MOTOR_RIGHT.id(), MotorType.kBrushless),
      new CANSparkMax(RobotMap.CAN.BACK_MOTOR_RIGHT.id(), MotorType.kBrushless)
    ),
    new Encoder(RobotMap.PWM.LEFT_ENCODER_IN.port(), RobotMap.PWM.LEFT_ENCODER_OUT.port(), false),
    new Encoder(RobotMap.PWM.RIGHT_ENCODER_IN.port(), RobotMap.PWM.RIGHT_ENCODER_OUT.port(), true),
    new AHRS(SPI.Port.kMXP)
  );


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
    return m_autoCommand;
  }
}
