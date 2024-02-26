package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;
public class PneumaticsSubsystem extends SubsystemBase {
    
    private final DoubleSolenoid shoulderSolenoid;

    private PneumaticsControlModule m_PneumaticsControlModule;

    private PneumaticHub hub;
    
    public PneumaticsSubsystem() {
        hub = new PneumaticHub(1);
        m_PneumaticsControlModule = new PneumaticsControlModule(1);
        shoulderSolenoid = hub.makeDoubleSolenoid(PneumaticsConstants.kShoulderForwardChannelPort, PneumaticsConstants.kShoulderReverseChannelPort);     
    }


    //Shoulder

    public void raiseShoulder() {
        shoulderSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void dropShoulder() {
        shoulderSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void toggleShoulder() {
        shoulderSolenoid.toggle();
    }

    public Command getRaiseShoulderCommand() {
        return this.runOnce(() -> {
            raiseShoulder();
        });
    }

    public Command getDropShoulderCommand() {
        return this.runOnce(() -> {
            dropShoulder();
        });
    }

    public Command getToggleShoulderCommand() {
        return this.runOnce(() -> {
            toggleShoulder();
        });
    }

    

    public boolean getShoulderRaised() {
        return shoulderSolenoid.get().equals(DoubleSolenoid.Value.kForward);
    }

    @Override
    public void periodic() {
        //TODO: add code for toggling compressor here
    }
}
