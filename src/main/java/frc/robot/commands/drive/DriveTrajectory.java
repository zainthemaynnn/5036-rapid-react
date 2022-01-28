package frc.robot.commands.drive;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.Drivetrain;

public class DriveTrajectory extends RamseteCommand {
    public DriveTrajectory(Drivetrain drivetrain, Trajectory trajectory) {
        super(
            trajectory,
            () -> drivetrain.pose,
            drivetrain.ramseteController,
            drivetrain.feedforward,
            drivetrain.kinematics,
            () -> drivetrain.wheelSpeeds,
            drivetrain.pidL,
            drivetrain.pidR,
            drivetrain::tankDriveVolts,
            drivetrain
        );

        drivetrain.resetOdometry(trajectory.getInitialPose());
    }
}
