// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.commands.OrElseCommand;
import frc.hid.PS4Controller;
import frc.hid.XBOXController;
import frc.math.VelocityTuner;
import frc.robot.commands.auto.ThreeBlue;
import frc.robot.commands.auto.FourBlue;
import frc.robot.commands.auto.Snipe;
import frc.robot.commands.drive.ArcadeDrive;
import frc.robot.commands.drive.CurvatureDrive;
import frc.robot.commands.drive.DriveAuto;
import frc.robot.commands.drive.DriveToPoint;
import frc.robot.commands.drive.FollowTrajectory;
import frc.robot.commands.drive.TurnAuto;
import frc.robot.commands.drive.TurnAutoCreative;
import frc.robot.commands.drive.FollowTrajectory.PoseData;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Blinkin.BlinkinColor;
import frc.robot.subsystems.Limelight.CameraMode;
import frc.robot.subsystems.Limelight.LEDState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final BuiltInAccelerometer accelerometer = new BuiltInAccelerometer();
  //private final PowerDistribution pdp = new PowerDistribution();

  // The robot's subsystems and commands are defined here...
  private final XBOXController driver = new XBOXController(RobotMap.Gamepad.DRIVER.port(), .04);

  private final Limelight limelight = new Limelight();
  private final Blinkin blinkin = new Blinkin(RobotMap.PWM.BLINKIN.port());

  private final CANSparkMax
    l1 = new CANSparkMax(RobotMap.CAN.BACK_MOTOR_LEFT.id(), MotorType.kBrushless),
    l2 = new CANSparkMax(RobotMap.CAN.FRONT_MOTOR_LEFT.id(), MotorType.kBrushless),
    r1 = new CANSparkMax(RobotMap.CAN.BACK_MOTOR_RIGHT.id(), MotorType.kBrushless),
    r2 = new CANSparkMax(RobotMap.CAN.FRONT_MOTOR_RIGHT.id(), MotorType.kBrushless);
  private final AHRS navx = new AHRS(SPI.Port.kMXP);

  public final Drivetrain drivetrain = new Drivetrain(
    l1, l2, r1, r2,
    l1.getEncoder(),
    r1.getEncoder(),
    navx
  );

  private final Intake intake = new Intake(
    new TalonSRX(RobotMap.CAN.INTAKE_UNO.id()),
    new TalonSRX(RobotMap.CAN.INTAKE_DOS.id())
  );

  public final Arm arm = new Arm(
    new CANSparkMax(RobotMap.CAN.ARM.id(), MotorType.kBrushless),
    new DigitalInput(RobotMap.DIO.ARM_BOTTOM_LIMIT_SWITCH.port())
  );

  private final Climber climber = new Climber(
    new VictorSPX(RobotMap.CAN.CLIMBER_UNO.id()),
    new VictorSPX(RobotMap.CAN.CLIMBER_DOS.id())
  );

  private final Command arcadeDrive = new ArcadeDrive(
    drivetrain,
    driver.leftStickY::value,
    driver.rightStickX::value
  );

  private final Command curvatureDrive = new CurvatureDrive(
    drivetrain,
    driver.leftStickY::value,
    driver.rightStickX::value,
    driver.leftTrigger::get
  );

  private final Command admitCargo = new StartEndCommand(
    () -> intake.runPercent(.70),
    () -> intake.stop(),
    intake
  );

  private final Command ejectCargo = new SequentialCommandGroup(
    new RunCommand(() -> intake.runPercent(1)).withTimeout(0.1),
    new StartEndCommand(
      () -> intake.runPercent(-1),
      () -> intake.stop(),
      intake
    )
  );

  private IdleMode idleMode = IdleMode.kBrake;
  private final ShuffleboardTab mainTab = Shuffleboard.getTab("Main");
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  private boolean stopped = true;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    DriveToPoint.init(drivetrain);
    drivetrain.setDefaultCommand(arcadeDrive);

    arm.setDefaultCommand(new RunCommand(
      () -> {
        if (!arm.overriden()) {
          if (!arm.isUp()) {
            arm.setPower(-.7);
            stopped = false;
          } else {
            arm.setPower(-.06);
            if (!stopped) {
              stopped = true;
            }
          }
        } else {
          if (!arm.isDown()) {
            arm.setPower(.6);
          } else {
            arm.setPower(.06);
          }
        }
      },
      arm
    ));

    blinkin.setDefaultCommand(new RunCommand(blinkin::display, blinkin));

    var timer = new Timer();
    intake.setDefaultCommand(new RunCommand(
      () -> {
        if (intake.hasBall() && timer.get() == 0.0 && !blinkin.containsColor(BlinkinColor.INTAKE)) {
          blinkin.addColor(BlinkinColor.INTAKE);
          limelight.setLEDState(LEDState.BLINK);
          timer.start();
        } else if (timer.get() >= 3.0) {
          blinkin.removeColor(BlinkinColor.INTAKE);
          limelight.setLEDState(LEDState.OFF);
          timer.stop();
          timer.reset();
        }
      },
      intake
    ));

    limelight.setCameraMode(CameraMode.DRIVER);
    limelight.setLEDState(LEDState.OFF);

    /*arm.setDefaultCommand(new ParallelCommandGroup(
      createArmDownCommand().perpetually(),
      admitCargo.perpetually()
    ).perpetually());*/

    double rampRate = 0.65;
    l1.setOpenLoopRampRate(rampRate);
    l2.setOpenLoopRampRate(rampRate);
    r1.setOpenLoopRampRate(rampRate);
    r2.setOpenLoopRampRate(rampRate);

    mainTab.add("Auto chooser", autoChooser);
    autoChooser.addOption("3 blue", new ThreeBlue(drivetrain, arm, intake));
    autoChooser.addOption("4 blue", new FourBlue(drivetrain, arm, intake));
    autoChooser.addOption("Snipe blue", new Snipe(drivetrain, arm, intake));

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
    VelocityTuner driveVelocityTuner = new VelocityTuner(
      drivetrain::getEncPosition,
      drivetrain::getEncVelocity,
      out -> drivetrain.arcadeDrive(out, 0.0)
    );

    VelocityTuner turnVelocityTuner = new VelocityTuner(
      drivetrain::getAngle,
      drivetrain::getAngularVelocity,
      out -> drivetrain.arcadeDrive(0.0, out)
    );

    driver.X.whileActiveOnce(new RunCommand(
      () -> driveVelocityTuner.driveVelocity(12.0),
      drivetrain
    ));

    driver.A.whileActiveOnce(new RunCommand(
      () -> turnVelocityTuner.driveVelocity(90.0),
      drivetrain
    ));

    // intake
    driver.rightTrigger.whileActiveOnce(admitCargo);

    driver.rightBumper.whenInactive(new ParallelCommandGroup(
      ejectCargo,
      new StartEndCommand(
        () -> blinkin.addColor(BlinkinColor.SHOOT),
        () -> blinkin.removeColor(BlinkinColor.SHOOT)
      )
    ).withTimeout(Constants.SHOOTER_TIMEOUT));

    driver.leftTrigger.whileActiveOnce(new StartEndCommand(
      () -> {
        arm.override(true);
        climber.extend();
      },
      climber::stop,
      climber,
      arm
    ));

    driver.leftBumper.whileActiveOnce(new StartEndCommand(
      climber::retract,
      climber::stop,
      climber
    ));

    // arm
    driver.rightTrigger
      .whileActiveOnce(new RunCommand(
        () -> {
          arm.override(false);
          if (!arm.isDown()) {
            arm.setPower(.60);
          } else {
            arm.setPower(.06);
          }
        },
        arm
      ))
      /*.whenActive(new InstantCommand(
        () -> {
          driver.setRumble(RumbleType.kLeftRumble, .5);
          driver.setRumble(RumbleType.kRightRumble, .5);
        }
      ))
      .whenInactive(new InstantCommand(
        () -> {
          driver.setRumble(RumbleType.kLeftRumble, 0);
          driver.setRumble(RumbleType.kRightRumble, 0);
        }
      ))*/;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    drivetrain.resetOdometry(new Pose2d());
    return autoChooser.getSelected();
  }
}
