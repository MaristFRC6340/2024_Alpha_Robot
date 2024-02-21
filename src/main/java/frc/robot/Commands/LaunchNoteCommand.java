package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class LaunchNoteCommand extends Command{

    IndexerSubsystem m_IndexerSubsystem;
    ShooterSubsystem m_ShooterSubsystem;

    public LaunchNoteCommand(ShooterSubsystem shooter, IndexerSubsystem indexer) {
        m_IndexerSubsystem = indexer;
        m_ShooterSubsystem = shooter;
        addRequirements(m_IndexerSubsystem, m_ShooterSubsystem);
    }

    @Override
    public void initialize() {
        m_IndexerSubsystem.runForwards();
        m_ShooterSubsystem.prepareLaunch();
    }

    @Override
    public void end(boolean interrupted) {
        m_IndexerSubsystem.stop();
        m_ShooterSubsystem.stop();
    }
}
