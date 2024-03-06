package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TheBassSubsystem;

public class TransferToIndexerCommand extends Command {
    

    private IndexerSubsystem m_IndexerSubsystem;

    private IntakeSubsystem m_IntakeSubsystem;


    int timer = 0;

    public TransferToIndexerCommand(IndexerSubsystem indexer, IntakeSubsystem intake) {
        m_IndexerSubsystem = indexer;
        m_IntakeSubsystem = intake;

        //m_ShooterSubsystem = shooter;
        addRequirements(m_IndexerSubsystem, m_IntakeSubsystem);
    }

    @Override
    public void initialize() {
        m_IndexerSubsystem.setPower(.3);

        m_IntakeSubsystem.intake();

        //m_ShooterSubsystem.setShooterPower(-.2);
    }

    @Override
    public void execute() {
        timer++;
    }

    @Override
    public void end(boolean interupted) {
        m_IndexerSubsystem.stop();

        m_IntakeSubsystem.stop();

        //m_ShooterSubsystem.stop();
    }
}
