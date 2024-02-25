package frc.robot.Commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToSourceCommand extends Command{
    private NetworkTable limTable;
    private NetworkTableEntry tx;
    private NetworkTableEntry ty;
    private NetworkTableEntry ledMode;

    private DriveSubsystem m_DriveSubsystem;
    
    public DriveToSourceCommand(DriveSubsystem drive) {
        m_DriveSubsystem = drive;
        addRequirements(m_DriveSubsystem);

        limTable = NetworkTableInstance.getDefault().getTable("limelight");
        ledMode = limTable.getEntry("ledMode");
        tx = limTable.getEntry("tx");
        ty = limTable.getEntry("ty");

    }

    @Override
    public void initialize() {
        ledMode.setDouble(3);
        tx = limTable.getEntry("tx");
        ty = limTable.getEntry("ty");
    }

    double xError;
    double yError;
    double rotError;
    @Override
    public void execute() {
        rotError = -m_DriveSubsystem.getPose().getRotation().getRadians();
            if(tx.exists()&& ty.exists()) {
                xError = (tx.getDouble(0));
                m_DriveSubsystem.drive(
                    .3, 
                    -1*xError*LimelightConstants.kPX, 
                    0, 
                    false, 
                    false);
            }
            else {
                m_DriveSubsystem.drive(
                    0, 
                    -1*xError*LimelightConstants.kPX, 
                    0, 
                    false, 
                    false);
            }
    }

    @Override
    public void end(boolean interrupted) {
        ledMode.setDouble(1);

        m_DriveSubsystem.drive(0, 0, 0, false, false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
