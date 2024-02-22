package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class LowLaunchNoteCommand extends SequentialCommandGroup{


    public LowLaunchNoteCommand(PneumaticsSubsystem pneum, ShooterSubsystem shooter, IndexerSubsystem indexer){
        addCommands(pneum.getDropShoulderCommand(), shooter.getPrepareLaunchCommand(), new WaitCommand(Constants.ShooterConstants.prepareLaunchDelay), indexer.getRunForwardCommand(), new WaitCommand(ShooterConstants.shootToRestDelay), indexer.getStopCommand(), shooter.getStopShooterCommand());
    }


    
}
