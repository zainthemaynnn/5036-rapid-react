// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Gamepad;
import frc.robot.commands.drive.ArcadeDrive;
import frc.robot.commands.drive.CurvatureDrive;
import frc.robot.commands.drive.DriveAuto;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.NetworkButton;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  //public static PowerDistribution pdp = new PowerDistribution();
  private BuiltInAccelerometer accelerometer = new BuiltInAccelerometer();

  // The robot's subsystems and commands are defined here...
  private final Gamepad driver = new Gamepad(RobotMap.Gamepad.DRIVER.port());
  private final Gamepad operator = new Gamepad(RobotMap.Gamepad.OPERATOR.port());

  private final Limelight limelight = new Limelight();
  // private final LedSubsystem ledSubsystem = new LedSubsystem();

  CANSparkMax L2 = new CANSparkMax(RobotMap.CAN.FRONT_MOTOR_LEFT.id(), MotorType.kBrushless);
  CANSparkMax L1 = new CANSparkMax(RobotMap.CAN.BACK_MOTOR_LEFT.id(), MotorType.kBrushless);
  CANSparkMax R2 = new CANSparkMax(RobotMap.CAN.FRONT_MOTOR_RIGHT.id(), MotorType.kBrushless);
  CANSparkMax R1 = new CANSparkMax(RobotMap.CAN.BACK_MOTOR_RIGHT.id(), MotorType.kBrushless);

  public final Drivetrain drivetrain = new Drivetrain(
    new MotorControllerGroup(
      L1, L2
    ),
    new MotorControllerGroup(
      R1, R2
    ),
    new Encoder(RobotMap.PWM.LEFT_ENCODER_IN.port(), RobotMap.PWM.LEFT_ENCODER_OUT.port(), false),
    new Encoder(RobotMap.PWM.RIGHT_ENCODER_IN.port(), RobotMap.PWM.RIGHT_ENCODER_OUT.port(), true),
    new AHRS(SPI.Port.kMXP)
  );

  private final Intake intake = new Intake(
    new TalonSRX(RobotMap.CAN.INTAKE_BOTTOM.id()),
    new TalonSRX(RobotMap.CAN.INTAKE_TOP.id())
  );

  public final Arm arm = new Arm(
    new CANSparkMax(RobotMap.CAN.ARM.id(), MotorType.kBrushless)
  );

  /*private final LedSubsystem ledSubsystem = new LedSubsystem(
    new Spark(RobotMap.PWM.BLINKIN.port())
  );*/

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

  private final Command admitCargo = new StartEndCommand(
    () -> intake.runPercent(1),
    () -> intake.stop(),
    intake
  );

  private final Command ejectCargo = new StartEndCommand(
    () -> intake.runPercent(-1),
    () -> intake.stop(),
    intake
  );

  private final Command setArm = new PIDCommand(
    new PIDController(.01, 0, 0),
    arm::getPosition,
    () -> arm.armTargetFromPercent(.9),
    arm::setPower,
    arm
  );

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drivetrain.setDefaultCommand(arcadeDrive); // TODO: decide drive style
    L1.setIdleMode(IdleMode.kBrake);
    L2.setIdleMode(IdleMode.kBrake);
    R1.setIdleMode(IdleMode.kBrake);
    R2.setIdleMode(IdleMode.kBrake);

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
    driver.getAxis(Gamepad.Axis.R2).whileActiveOnce(admitCargo);

    SmartDashboard.putNumber("P", 0);
    SmartDashboard.putNumber("I", 0);
    SmartDashboard.putNumber("D", 0);

    var pidArmDown = new PIDController(0, 0, 0);
    pidArmDown.setTolerance(3);
    driver.getAxis(Gamepad.Axis.R2).whileActiveOnce(new PIDCommand(
      pidArmDown,
      arm::getPosition,
      () -> 90,
      arm::setPower,
      arm
    ));

    var pidArmUp = new PIDController(0, 0, 0);
    pidArmUp.setTolerance(3);
    driver.getAxis(Gamepad.Axis.R2).whenInactive(new PIDCommand(
      pidArmUp,
      arm::getPosition,
      () -> 31,
      arm::setPower,
      arm
    ));

    CommandScheduler.getInstance().schedule(new RunCommand(
      () -> {
        double p = SmartDashboard.getNumber("P", 0), i = SmartDashboard.getNumber("I", 0), d = SmartDashboard.getNumber("D", 0);
        pidArmDown.setPID(p, i, d);
        pidArmUp.setPID(p, i, d);
      }
    ));

    driver.getButton(Gamepad.Button.R1).whenReleased(ejectCargo.withTimeout(Constants.SHOOTER_TIMEOUT));
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
