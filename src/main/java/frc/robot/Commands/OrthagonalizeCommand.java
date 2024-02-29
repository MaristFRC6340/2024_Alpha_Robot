package frc.robot.Commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.DriveSubsystem;

public class OrthagonalizeCommand extends Command{
  

    private DriveSubsystem m_DriveSubsystem;
    
    public OrthagonalizeCommand(DriveSubsystem drive) {
        m_DriveSubsystem = drive;
        addRequirements(m_DriveSubsystem);

        

    }

    @Override
    public void initialize() {
       
    }

   
    double rotError;
    @Override
    public void execute() {
                
                rotError = -m_DriveSubsystem.getGyroAngle().getRadians()*LimelightConstants.kPRot*50;
                if(rotError < 0)rotError-=.1;
                else rotError+=.1;

                if(Math.abs(rotError)-.1 < .01)rotError=0;
                System.out.println(rotError);

                m_DriveSubsystem.drive(
                    0,0,
                    rotError, 
                    false, 
                    false);
            
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        if(rotError==0) {
            return true;
        }
        return false;
    }
}
