package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
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
    int timer = 0;

    public ShootOnTheFlyCommand(IndexerSubsystem indexer, ShooterSubsystem shooter, PneumaticsSubsystem pneumatics, DriveSubsystem drive, DoubleSupplier supplier, DoubleSupplier leftXSupplier, DoubleSupplier leftYSupplier, DoubleSupplier rightXSupplier) {
        m_IndexerSubsystem = indexer;
        m_ShooterSubsystem = shooter;
        m_PneumaticsSubsystem = pneumatics;
        m_DriveSubsystem = drive;
        addRequirements(indexer, shooter, pneumatics, drive);

        controlSupplier = supplier;

        this.leftXSupplier = leftXSupplier;
        this.leftYSupplier = leftYSupplier;
        this.rightXSupplier = rightXSupplier;
    }

    @Override
    public void initialize() {
        m_ShooterSubsystem.prepareLaunch();

        m_PneumaticsSubsystem.dropShoulder();

        m_IndexerSubsystem.stop();
    }

    PIDController targetingPID = new PIDController(.01, 0, 0.000001);


    @Override
    public void execute() {
        double speedControl = controlSupplier.getAsDouble();
        
        if(LimelightHelpers.getTV("limelight")) {
            double leftX = leftXSupplier.getAsDouble();
            double leftY = leftYSupplier.getAsDouble();

            double rotSpeed = targetingPID.calculate(LimelightHelpers.getTX("limelight"), 0);

            m_DriveSubsystem.drive(
            MathUtil.applyDeadband(-leftY*speedControl, .06),
            MathUtil.applyDeadband(-leftX*speedControl, .06), 
            MathUtil.applyDeadband(rotSpeed, .06), true, false);
        }
        else {
            double leftX = leftXSupplier.getAsDouble();
            double leftY = leftYSupplier.getAsDouble();
            double rightX = rightXSupplier.getAsDouble();

            m_DriveSubsystem.drive(-leftX*speedControl, -leftY*speedControl, -rightX, true, false);
        }

        timer++;
    }





    double lastUnadjustedTY = LimelightHelpers.getTY("limelight");
    double lastUnadjustedTS = 0;

    @Override
    public boolean isFinished() {
        if(LimelightHelpers.getTV("limelight") && lastUnadjustedTS !=timer) {
            double currentTY = LimelightHelpers.getTY("limelight");
            double currentTS = timer*20;

            double adjustedTY = ((currentTY-lastUnadjustedTY)/(currentTS-lastUnadjustedTS))*(LimelightConstants.kShooterDelight)+currentTY;
            
            SmartDashboard.putNumber("Velocity Adjusted TY", adjustedTY);

            lastUnadjustedTS = currentTS;
            lastUnadjustedTY = currentTY;

            return adjustedTY > -10 && adjustedTY < -9 && Math.abs(LimelightHelpers.getTX("limelight"))<2.5;
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
