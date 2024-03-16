package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeUntilNoteCommand extends Command{
    
    private IntakeSubsystem m_IntakeSubsystem;
    private boolean hasNoteInitial;

    public IntakeUntilNoteCommand(IntakeSubsystem intake ,boolean initalStateHasNote) {
        m_IntakeSubsystem = intake;
        addRequirements(intake);

        hasNoteInitial=initalStateHasNote;
    }

    @Override
    public void initialize() {
        m_IntakeSubsystem.intake();
    }

    @Override
    public void end(boolean interrupted) {
        m_IntakeSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        if(!hasNoteInitial) return m_IntakeSubsystem.hasNote();
        else return false;
    }
}
