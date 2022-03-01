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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.hid.PS4Controller;
import frc.hid.XBOXController;
import frc.robot.commands.OrElseCommand;
import frc.robot.commands.drive.ArcadeDrive;
import frc.robot.commands.drive.CurvatureDrive;
import frc.robot.commands.drive.DriveAuto;
import frc.robot.commands.drive.FollowTrajectory;
import frc.robot.commands.drive.TurnAuto;
import frc.robot.commands.drive.FollowTrajectory.PoseData;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Blinkin;
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
import edu.wpi.first.wpilibj2.command.StartEndCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private BuiltInAccelerometer accelerometer = new BuiltInAccelerometer();

  // The robot's subsystems and commands are defined here...
  private final XBOXController driver = new XBOXController(RobotMap.Gamepad.DRIVER.port(), .04);

  private final Limelight limelight = new Limelight();
  private final Blinkin blinkin = new Blinkin(RobotMap.PWM.BLINKIN.port());

  private final CANSparkMax L1 = new CANSparkMax(RobotMap.CAN.BACK_MOTOR_LEFT.id(), MotorType.kBrushless);
  private final CANSparkMax L2 = new CANSparkMax(RobotMap.CAN.FRONT_MOTOR_LEFT.id(), MotorType.kBrushless);
  private final CANSparkMax R1 = new CANSparkMax(RobotMap.CAN.BACK_MOTOR_RIGHT.id(), MotorType.kBrushless);
  private final CANSparkMax R2 = new CANSparkMax(RobotMap.CAN.FRONT_MOTOR_RIGHT.id(), MotorType.kBrushless);
  private final AHRS navx = new AHRS(SPI.Port.kMXP);

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
    new TalonSRX(RobotMap.CAN.INTAKE_UNO.id()),
    new TalonSRX(RobotMap.CAN.INTAKE_DOS.id()),
    blinkin,
    limelight
  );

  /*public final Arm arm = new Arm(
    new CANSparkMax(RobotMap.CAN.ARM.id(), MotorType.kBrushless),
    new DigitalInput(RobotMap.DIO.ARM_BOTTOM_LIMIT_SWITCH.port())
  );*/

  private final Command arcadeDrive = new ArcadeDrive(
    drivetrain,
    driver.leftStickY::value,
    driver.rightStickX::value
  );

  private final Command curvatureDrive = new CurvatureDrive(
    drivetrain,
    driver.leftStickY::value,
    driver.rightStickX::value
  );

  private final Command admitCargo = new StartEndCommand(
    () -> intake.runPercent(.6),
    () -> intake.stop(),
    intake
  );

  private final Command ejectCargo = new StartEndCommand(
    () -> intake.runPercent(-1),
    () -> intake.stop(),
    intake
  );

  private final Command ejectCargoShort = new StartEndCommand(
    () -> intake.runPercent(-1),
    () -> intake.stop(),
    intake
  );

  private IdleMode idleMode = IdleMode.kBrake;

  private void setIdleMode(IdleMode m) {
    L1.setIdleMode(m); 
    L2.setIdleMode(m);
    R1.setIdleMode(m);
    R2.setIdleMode(m);
  }

  private double last = 0.0;
  private boolean isup = true;
  private boolean stopped = true;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drivetrain.setDefaultCommand(arcadeDrive);
    setIdleMode(idleMode);

    /*arm.setDefaultCommand(new RunCommand(
      () -> {
        if (!arm.isUp()) {
          arm.setPower(-.6);
          intake.runPercent(.6);
          stopped = false;
        } else {
          arm.setPower(-.2);
          if (!stopped) {
            intake.runPercent(0);
            stopped = true;
          }
        }
      },
      arm
    ));*/
  
    /*arm.setDefaultCommand(new OrElseCommand(
      createArmDownCommand(),
      createArmUpCommand(),
      driver.rightTrigger::get
    ));*/
    blinkin.setDefaultCommand(new RunCommand(blinkin::display, blinkin));

    var timer = new Timer();
    intake.setDefaultCommand(new RunCommand(
      () -> {
        if (timer.get() == 0.0 && intake.hasBall() && /*arm.isDown() &&*/ !blinkin.containsColor(BlinkinColor.INTAKE)) {
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
    L1.setClosedLoopRampRate(rampRate);
    L2.setClosedLoopRampRate(rampRate);
    R1.setClosedLoopRampRate(rampRate);
    R2.setClosedLoopRampRate(rampRate);

    // Configure the button bindings
    configureButtonBindings();
  }

  public Command createAutoCommand() {
    return new TurnAuto(drivetrain, 90.0);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    driver.A.whileActiveOnce(getAutonomousCommand());
    //driver.X.whenInactive(new InstantCommand(() -> drivetrain.resetOdometry(new Pose2d()), drivetrain));
    driver.Y.whenActive(new InstantCommand(
      () -> {
        if (idleMode == IdleMode.kBrake) {
          idleMode = IdleMode.kCoast;
        } else {
          idleMode = IdleMode.kBrake;
        }
        setIdleMode(idleMode);
      },
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

    driver.leftBumper.whenInactive(new ParallelCommandGroup(
      ejectCargoShort
      /*new StartEndCommand(
        () -> blinkin.addColor(BlinkinColor.SHOOT),
        () -> blinkin.removeColor(BlinkinColor.SHOOT)
      )*/
    ).withTimeout(.1));

    // arm
    //driver.rightTrigger
      /*.whileActiveOnce(new RunCommand(
        () -> {
          if (!arm.isDown()) {
            arm.setPower(.6);
          } else {
            arm.setPower(.2);
          }
        },
        arm
      ))*/
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
      ))*///;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    ArrayList<PoseData> path = new ArrayList<>();
    drivetrain.resetOdometry(new Pose2d());
    /*path.add(new PoseData(
      new Pose2d(
        new Translation2d(-10.0, 20.0),
        new Rotation2d(Math.toRadians(30))
      ),
      1.0,
      1.0,
      true
    ));
    path.add(new PoseData(
      new Pose2d(
        new Translation2d(0.0, 40.0),
        new Rotation2d(Math.toRadians(-30))
      ),
      1.0,
      1.0,
      false
    ));
    path.add(new PoseData(
      new Pose2d(
        new Translation2d(17.90, 60.92),
        new Rotation2d(Math.toRadians(-90))
      ),
      1.0,
      1.0,
      false
    ));*/
    path.add(new PoseData(
      new Pose2d(
        new Translation2d(40.0, 100.0),
        new Rotation2d(Math.toRadians(-90))
      ),
      1.0,
      0.1,
      true
    ));
    return new FollowTrajectory(
      drivetrain,
      path
    );
  }
}
