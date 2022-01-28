// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.GroupMotorControllers;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Gamepad;
import frc.robot.Gamepad.Axis;
import frc.robot.Gamepad.Button;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.conveyor.EjectCargo;
import frc.robot.commands.drive.ArcadeDrive;
import frc.robot.commands.drive.CurvatureDrive;
import frc.robot.commands.drive.DriveAuto;
import frc.robot.commands.drive.TurnAuto;
import frc.robot.commands.intake.AdmitCargo;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
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

  private final Gamepad driver = new Gamepad(RobotMap.Gamepad.DRIVER.port());
  private final Gamepad operator = new Gamepad(RobotMap.Gamepad.OPERATOR.port());

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

  private final Intake m_intake = new Intake(
    new TalonSRX(RobotMap.CAN.INTAKE_BOTTOM.id()),
    new TalonSRX(RobotMap.CAN.INTAKE_TOP.id())
  );

  private final ArcadeDrive m_arcadeDrive = new ArcadeDrive(
    m_drivetrain,
    () -> driver.getAxisValue(Gamepad.Axis.LEFT_Y, .02),
    () -> driver.getAxisValue(Gamepad.Axis.RIGHT_X, .02)
  );

  private final CurvatureDrive m_curvatureDrive = new CurvatureDrive(
    m_drivetrain,
    () -> driver.getAxisValue(Gamepad.Axis.LEFT_Y, .02),
    () -> driver.getAxisValue(Gamepad.Axis.RIGHT_X, .02)
  );

  private final AdmitCargo m_admitCargo = new AdmitCargo(m_intake);

  private final EjectCargo m_ejectCargo = new EjectCargo();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_drivetrain.setDefaultCommand(m_arcadeDrive); // TODO: decide drive style

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
    operator.getAxis(Axis.R2).whileActiveOnce(m_admitCargo);
    operator.getButton(Button.R1).whenReleased(m_ejectCargo);
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
