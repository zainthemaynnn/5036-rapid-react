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
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.hid.PS4Controller;
import frc.hid.XBOXController;
import frc.robot.commands.drive.ArcadeDrive;
import frc.robot.commands.drive.CurvatureDrive;
import frc.robot.commands.drive.DriveAuto;
import frc.robot.commands.drive.FollowTrajectory;
import frc.robot.commands.drive.TurnAuto;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
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
  private final XBOXController driver = new XBOXController(RobotMap.Gamepad.DRIVER.port());

  private final Limelight limelight = new Limelight();
  // private final LedSubsystem ledSubsystem = new LedSubsystem();

  CANSparkMax L2 = new CANSparkMax(RobotMap.CAN.FRONT_MOTOR_LEFT.id(), MotorType.kBrushless);
  CANSparkMax L1 = new CANSparkMax(RobotMap.CAN.BACK_MOTOR_LEFT.id(), MotorType.kBrushless);
  CANSparkMax R2 = new CANSparkMax(RobotMap.CAN.FRONT_MOTOR_RIGHT.id(), MotorType.kBrushless);
  CANSparkMax R1 = new CANSparkMax(RobotMap.CAN.BACK_MOTOR_RIGHT.id(), MotorType.kBrushless);
  AHRS navx = new AHRS(SPI.Port.kMXP);

  public final Drivetrain drivetrain = new Drivetrain(
    new MotorControllerGroup(
      L1, L2
    ),
    new MotorControllerGroup(
      R1, R2
    ),
    L1.getEncoder(),
    R1.getEncoder(),
    navx
  );

  private final Intake intake = new Intake(
    new TalonSRX(RobotMap.CAN.INTAKE_BOTTOM.id()),
    new TalonSRX(RobotMap.CAN.INTAKE_TOP.id())
  );

  public final Arm arm = new Arm(
    new CANSparkMax(RobotMap.CAN.ARM.id(), MotorType.kBrushless),
    new DigitalInput(RobotMap.DIO.ARM_BOTTOM_LIMIT_SWITCH.port())
  );

  private final Blinkin blinkin = new Blinkin(RobotMap.PWM.BLINKIN.port());

  /*private final LedSubsystem ledSubsystem = new LedSubsystem(
    new Spark(RobotMap.PWM.BLINKIN.port())
  );*/

  private final ArcadeDrive arcadeDrive = new ArcadeDrive(
    drivetrain,
    driver.leftStickY::value,
    driver.rightStickX::value
  );

  private final CurvatureDrive curvatureDrive = new CurvatureDrive(
    drivetrain,
    driver.leftStickY::value,
    driver.rightStickX::value
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

  private final PIDController
    pidArmUp = new PIDController(0.015, 0.001, 1.5);

  private final Command armUp = new PIDCommand(
    pidArmUp,
    arm::getPosition,
    () -> 31,
    power -> arm.setPower(Math.abs(power) < Constants.ARM_MAX_UP ? power : Constants.ARM_MAX_UP * Math.signum(power)),
    arm
  );

  private Command createArmDownCommand() {
    var pidArmDown = new PIDController(0.015, 0.001, 1.5);
    pidArmDown.setTolerance(3.0);
    return new PIDCommand(
      pidArmDown,
      arm::getPosition,
      () -> 90,
      power -> {
        if (arm.isDown() && arm.getLimSwitch()) {
          arm.setPower(.2);
        } else {
          arm.setPower(power);
        }
      },
      arm
    );
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drivetrain.setDefaultCommand(arcadeDrive); // TODO: decide drive style

    pidArmUp.setTolerance(3.0);

    /*final float YAW_TIP_POINT = 25;
    try {
      arm.setDefaultCommand(new ConditionalCommand(
        createArmDownCommand(),
        armUp,
        () -> Math.abs(navx.getYaw()) > YAW_TIP_POINT
      ));
    } catch (NullPointerException e) {
      // TODO: this is a terrible idea. but why does it happen?
    }*/
    arm.setDefaultCommand(armUp);

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
    driver.A.whileActiveOnce(getAutonomousCommand());
    driver.X.whenInactive(new InstantCommand(() -> drivetrain.resetOdometry(new Pose2d()), drivetrain));

    // intake
    driver.rightTrigger.whileActiveOnce(admitCargo);
    driver.rightBumper.whenInactive(ejectCargo.withTimeout(Constants.SHOOTER_TIMEOUT));
    //blinkin.set(-.97);

    // arm
    driver.rightTrigger.whileActiveOnce(createArmDownCommand());
    driver.rightTrigger.whenActive(new InstantCommand(() -> {
      driver.setRumble(RumbleType.kLeftRumble, .5);
      driver.setRumble(RumbleType.kRightRumble, .5);
    }));
    driver.rightTrigger.whenInactive(new InstantCommand(() -> {
      driver.setRumble(RumbleType.kLeftRumble, .0);
      driver.setRumble(RumbleType.kRightRumble, .0);
    }));
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
