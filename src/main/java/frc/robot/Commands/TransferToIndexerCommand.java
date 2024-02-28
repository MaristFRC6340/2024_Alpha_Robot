package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TheBassSubsystem;

public class TransferToIndexerCommand extends Command {
    
    private TheBassSubsystem m_TheBassSubsystem;

    private IndexerSubsystem m_IndexerSubsystem;

    private IntakeSubsystem m_IntakeSubsystem;


    int timer = 0;

    public TransferToIndexerCommand(TheBassSubsystem bass, IndexerSubsystem indexer, IntakeSubsystem intake) {
        m_TheBassSubsystem = bass;
        m_IndexerSubsystem = indexer;
        m_IntakeSubsystem = intake;

        addRequirements(m_TheBassSubsystem, m_IndexerSubsystem, m_IntakeSubsystem);
    }

    @Override
    public void initialize() {
        m_TheBassSubsystem.goToTransfer();

        m_IndexerSubsystem.runForwards();

        m_IntakeSubsystem.intake();
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interupted) {
        m_TheBassSubsystem.stop();

        m_IndexerSubsystem.stop();

        m_IntakeSubsystem.stop();
    }
}
