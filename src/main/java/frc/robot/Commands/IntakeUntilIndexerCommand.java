package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeUntilIndexerCommand extends Command{
    
    private IndexerSubsystem m_IndexerSubsystem;
    private IntakeSubsystem m_IntakeSubsystem;

    public IntakeUntilIndexerCommand(IndexerSubsystem indexer, IntakeSubsystem intake) {
        m_IndexerSubsystem = indexer;
        m_IntakeSubsystem = intake;

        addRequirements(indexer, intake);
    }

    @Override
    public void initialize() {
        m_IndexerSubsystem.getRunForwardCommand();
        m_IntakeSubsystem.getIntakeCommand();
    }

    @Override
    public void end(boolean interrupted) {
        m_IndexerSubsystem.stop();
        m_IntakeSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return m_IndexerSubsystem.hasNote();
    }
}
