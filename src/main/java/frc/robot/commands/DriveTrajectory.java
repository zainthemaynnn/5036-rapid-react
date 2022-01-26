package frc.robot.commands;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class DriveTrajectory extends RamseteCommand {
    public DriveTrajectory(Drivetrain drivetrain, Trajectory trajectory) {
        super(
            trajectory,
            () -> drivetrain.pose,
            new RamseteController(),
            drivetrain.feedforward,
            drivetrain.kinematics,
            () -> drivetrain.wheelSpeeds,
            drivetrain.pidL,
            drivetrain.pidR,
            (voltageL, voltageR) -> {
                // TODO: what is this for?
            },
            drivetrain
        );
    }

    public void execute() {

    }

    public boolean isFinished() {
        return false;
    }
}
