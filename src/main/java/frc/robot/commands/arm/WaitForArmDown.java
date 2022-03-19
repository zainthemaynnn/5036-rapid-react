package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Arm;

public class WaitForArmDown extends RunCommand {
    public WaitForArmDown(Arm arm) {
        super(() -> {});
        withInterrupt(arm::isDown);
    }
}
