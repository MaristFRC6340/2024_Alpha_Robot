package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootOnTheFlyCommand extends Command{
    
    private IndexerSubsystem m_IndexerSubsystem;
    private ShooterSubsystem m_ShooterSubsystem;
    private PneumaticsSubsystem m_PneumaticsSubsystem;
    private DriveSubsystem m_DriveSubsystem;

    private DoubleSupplier leftXSupplier;
    private DoubleSupplier leftYSupplier;
    private DoubleSupplier rightXSupplier;

    private DoubleSupplier controlSupplier;

    public ShootOnTheFlyCommand(IndexerSubsystem indexer, ShooterSubsystem shooter, PneumaticsSubsystem pneumatics, DoubleSupplier supplier, DoubleSupplier leftXSupplier, DoubleSupplier leftYSupplier, DoubleSupplier rightXSupplier) {
        m_IndexerSubsystem = indexer;
        m_ShooterSubsystem = shooter;
        m_PneumaticsSubsystem = pneumatics;

        addRequirements(indexer, shooter, pneumatics);

        controlSupplier = supplier;

        this.leftXSupplier = leftXSupplier;
        this.leftYSupplier = leftYSupplier;
        this.rightXSupplier = rightXSupplier;
    }

    @Override
    public void initialize() {
        m_ShooterSubsystem.getPrepareLaunchCommand();

        m_PneumaticsSubsystem.getDropShoulderCommand();

        m_IndexerSubsystem.stop();
    }

    PIDController targetingPID = new PIDController(.01, .001, .001);


    @Override
    public void execute() {
        double speedControl = controlSupplier.getAsDouble();
        
        if(LimelightHelpers.getTV("limelight")) {
            double leftX = leftXSupplier.getAsDouble();
            double leftY = leftXSupplier.getAsDouble();

            double rotSpeed = targetingPID.calculate(LimelightHelpers.getTX("limelight"), 0);

            m_DriveSubsystem.drive(-leftX*speedControl, -leftY*speedControl, -rotSpeed, true, false);
        }
        else {
            double leftX = leftXSupplier.getAsDouble();
            double leftY = leftYSupplier.getAsDouble();
            double rightX = rightXSupplier.getAsDouble();

            m_DriveSubsystem.drive(-leftX*speedControl, -leftY*speedControl, -rightX, true, false);
        }
    }





    double lastUnadjustedTY = LimelightHelpers.getTY("limelight");
    double lastUnadjustedTS = LimelightHelpers.getTS("limelight");

    @Override
    public boolean isFinished() {
        if(LimelightHelpers.getTV("limelight") && lastUnadjustedTS !=LimelightHelpers.getTS("limelight")) {
            double currentTY = LimelightHelpers.getTY("limelight");
            double currentTS = LimelightHelpers.getTS("limelight");

            double adjustedTY = ((currentTY-lastUnadjustedTY)/(currentTS-lastUnadjustedTS))*(LimelightConstants.kShooterDelight)+currentTY;
            
            SmartDashboard.putNumber("Velocity Adjusted TY", adjustedTY);

            lastUnadjustedTS = currentTS;
            lastUnadjustedTY = currentTY;

            return adjustedTY > -7 && adjustedTY < -6 && Math.abs(LimelightHelpers.getTX("limelight"))<2;
        }

        
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        if(interrupted) {
            m_ShooterSubsystem.stop();
        }
    }
}
