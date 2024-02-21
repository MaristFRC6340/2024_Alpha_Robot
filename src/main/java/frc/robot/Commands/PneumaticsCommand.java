package frc.robot.Commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.Constants.OIConstants;
public class PneumaticsCommand extends Command{
    
    private PneumaticsSubsystem m_PneumaticsSubsystem;

    private XboxController shooterController = new XboxController(OIConstants.kActuatorControllerPort);
    public PneumaticsCommand(PneumaticsSubsystem pneumatics) {
        m_PneumaticsSubsystem = pneumatics;
        addRequirements(m_PneumaticsSubsystem);
    }

    @Override
    public void execute() {
        if(shooterController.getLeftBumperPressed()) {
            m_PneumaticsSubsystem.dropBass();
        }
        else if(shooterController.getRightBumperPressed()) {
            m_PneumaticsSubsystem.raiseBass();
        }
    }
}
