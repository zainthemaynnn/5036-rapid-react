package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.LedSubsystem;

public class LedCommand implements Command {

    private final LedSubsystem m_ledSubsystem;
    private Timer ledTimer = new Timer();
    private final double MATCH_TOTAL_TIME = 120;

    public LedCommand(LedSubsystem ledSubsystem) {
        m_ledSubsystem = ledSubsystem;
        ledTimer.reset();
    }

    @Override
    public void initialize() {
        ledTimer.start();
        m_ledSubsystem.bootUpFlash();
    }

    @Override
    public void execute() {
        if (MATCH_TOTAL_TIME - ledTimer.get() == 30) {
            m_ledSubsystem.startFlashing();
        }
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        if (MATCH_TOTAL_TIME - ledTimer.get() <= 0) {
            return true;
        }
        return false;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(m_ledSubsystem);
    }
    
}
