// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.GroupMotorControllers;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Gamepad;
import frc.robot.Gamepad.Axis;
import frc.robot.Gamepad.Button;
import frc.robot.commands.arm.LowerArm;
import frc.robot.commands.conveyor.EjectCargo;
import frc.robot.commands.drive.ArcadeDrive;
import frc.robot.commands.drive.CurvatureDrive;
import frc.robot.commands.drive.DriveAuto;
import frc.robot.commands.drive.TurnAuto;
import frc.robot.commands.intake.AdmitCargo;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  PowerDistribution pdp = new PowerDistribution();
  BuiltInAccelerometer accelerometer = new BuiltInAccelerometer();

  // The robot's subsystems and commands are defined here...
  private final Gamepad driver = new Gamepad(RobotMap.Gamepad.DRIVER.port());
  private final Gamepad operator = new Gamepad(RobotMap.Gamepad.OPERATOR.port());

  private final Limelight limelight = new Limelight();
  // private final LedSubsystem ledSubsystem = new LedSubsystem();

  private final Drivetrain drivetrain = new Drivetrain(
    new MotorControllerGroup(
      //new CANSparkMax(RobotMap.CAN.FRONT_MOTOR_LEFT.id(), MotorType.kBrushless),
      new CANSparkMax(RobotMap.CAN.BACK_MOTOR_LEFT.id(), MotorType.kBrushless)
    ),
    new MotorControllerGroup(
      //new CANSparkMax(RobotMap.CAN.FRONT_MOTOR_RIGHT.id(), MotorType.kBrushless),
      new CANSparkMax(RobotMap.CAN.BACK_MOTOR_RIGHT.id(), MotorType.kBrushless)
    ),
    new Encoder(RobotMap.PWM.LEFT_ENCODER_IN.port(), RobotMap.PWM.LEFT_ENCODER_OUT.port(), false),
    new Encoder(RobotMap.PWM.RIGHT_ENCODER_IN.port(), RobotMap.PWM.RIGHT_ENCODER_OUT.port(), true),
    new AHRS(SPI.Port.kMXP)
  );

  private final Intake intake = new Intake(
    new TalonSRX(RobotMap.CAN.INTAKE_BOTTOM.id()),
    new TalonSRX(RobotMap.CAN.INTAKE_TOP.id())
  );

  private final Arm arm = new Arm(
    new PWMSparkMax(RobotMap.CAN.ARM.id())
  );

  private final LedSubsystem ledSubsystem = new LedSubsystem(
    new Spark(RobotMap.PWM.BLINKIN.port())
  );

  private final ArcadeDrive arcadeDrive = new ArcadeDrive(
    drivetrain,
    () -> driver.getAxisValue(Gamepad.Axis.LEFT_Y),
    () -> driver.getAxisValue(Gamepad.Axis.RIGHT_X)
  );

  private final CurvatureDrive curvatureDrive = new CurvatureDrive(
    drivetrain,
    () -> driver.getAxisValue(Gamepad.Axis.LEFT_Y),
    () -> driver.getAxisValue(Gamepad.Axis.RIGHT_X)
  );

  private final AdmitCargo admitCargo = new AdmitCargo(intake, arm);

  private final EjectCargo ejectCargo = new EjectCargo();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drivetrain.setDefaultCommand(arcadeDrive); // TODO: decide drive style

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
    operator.getAxis(Axis.R2).whileActiveOnce(admitCargo);
    operator.getButton(Button.R1).whenReleased(ejectCargo);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new DriveAuto(drivetrain, 1.0, false);
  }
}
