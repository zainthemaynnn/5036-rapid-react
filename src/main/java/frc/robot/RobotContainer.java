// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.commands.drive.ArcadeDrive;
import frc.robot.commands.drive.CurvatureDrive;
import frc.robot.commands.drive.DriveAuto;
import frc.robot.commands.drive.FollowTrajectory;
import frc.robot.commands.drive.TurnAuto;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // public static PowerDistribution pdp = new PowerDistribution();
  private BuiltInAccelerometer accelerometer = new BuiltInAccelerometer();

  // The robot's subsystems and commands are defined here...
  private final Gamepad driver = new Gamepad(RobotMap.Gamepad.DRIVER.port());

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
    L1.getEncoder(),
    R1.getEncoder(),
    new AHRS(SPI.Port.kMXP)
  );

  private final Intake intake = new Intake(
    new TalonSRX(RobotMap.CAN.INTAKE_BOTTOM.id()),
    new TalonSRX(RobotMap.CAN.INTAKE_TOP.id())
  );

  public final Arm arm = new Arm(
    new CANSparkMax(RobotMap.CAN.ARM.id(), MotorType.kBrushless),
    new DigitalInput(RobotMap.DIO.ARM_BOTTOM_LIMIT_SWITCH.port())
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
    () -> intake.runPercent(.75),
    () -> intake.stop(),
    intake
  );

  private final Command ejectCargo = new StartEndCommand(
    () -> intake.runPercent(-1),
    () -> intake.stop(),
    intake
  );

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drivetrain.setDefaultCommand(arcadeDrive); // TODO: decide drive style
    //arm.setDefaultCommand(new RunCommand(arm::updateDashboard, arm));
    IdleMode idleMode = IdleMode.kBrake;
    L1.setIdleMode(idleMode);
    L2.setIdleMode(idleMode);
    R1.setIdleMode(idleMode);
    R2.setIdleMode(idleMode);
    double rampRate = 0.65;
    L1.setClosedLoopRampRate(rampRate);
    L2.setClosedLoopRampRate(rampRate);
    R1.setClosedLoopRampRate(rampRate);
    R2.setClosedLoopRampRate(rampRate);

    // Configure the button bindings
    configureButtonBindings();

    Spark blinkin = new Spark(RobotMap.PWM.BLINKIN.port());
    blinkin.set(.45);
  }

  public Command createAutoCommand() {
    return new TurnAuto(drivetrain, 90);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    driver.getButton(Gamepad.Button.GREEN).whileActiveOnce(getAutonomousCommand());
    driver.getButton(Gamepad.Button.BLUE).whenReleased(new InstantCommand(() -> drivetrain.resetOdometry(new Pose2d()), drivetrain));

    // intake
    driver.getAxis(Gamepad.Axis.R2).whileActiveOnce(admitCargo);
    driver.getButton(Gamepad.Button.R1).whenReleased(ejectCargo.withTimeout(Constants.SHOOTER_TIMEOUT));

    // arm
    var pidArmDown = new PIDController(0.015, 0.001, 1.5);
    pidArmDown.setTolerance(3.0);
    driver.getAxis(Gamepad.Axis.R2).whileActiveOnce(new PIDCommand(
      pidArmDown,
      arm::getPosition,
      () -> 98,
      power -> {
        if (arm.isDown() && arm.getLimSwitch()) {
          arm.setPower(.2);
        } else {
          arm.setPower(power);
        }
      },
      arm
    ));

    var pidArmUp = new PIDController(0.015, 0.001, 1.5);
    pidArmUp.setTolerance(3.0);
    driver.getAxis(Gamepad.Axis.R2).whenInactive(new PIDCommand(
      pidArmUp,
      arm::getPosition,
      () -> 31,
      arm::setPower,
      arm
    ));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    ArrayList<Pose2d> path = new ArrayList<>();
    drivetrain.resetOdometry(new Pose2d());
    path.add(new Pose2d(
      new Translation2d(17.90, 60.92),
      new Rotation2d(Math.toRadians(90))
    ));
    return new FollowTrajectory(
      drivetrain,
      path,
      1.0
    );
  }
}
