package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.DriveSubsystem;


//NOTE: DOES NOT WORK! DO NOT USE! WORK IN PROGRESS!
public class LowLatencyDriveTargetLockCommand extends Command{
    private final DriveSubsystem m_robotDrive;

    private double speedControl=DriveConstants.kSpeedControl;


    DoubleSupplier getLeftX;
    DoubleSupplier getLeftY;
    DoubleSupplier getRightX;
    NetworkTableEntry tX;
    NetworkTableEntry tS;
    NetworkTable limTable;

    Timer time = new Timer();


    double turnPower = 0;

    private boolean fieldCentric = true;

    private DoubleSupplier speedSupplier;


    public LowLatencyDriveTargetLockCommand(DriveSubsystem drive, DoubleSupplier getLeftX, DoubleSupplier getLeftY, DoubleSupplier getRightX, DoubleSupplier speedSupplier) {
        this.getLeftX = getLeftX;
        this.getLeftY = getLeftY;
        this.getRightX = getRightX;

        this.limTable = NetworkTableInstance.getDefault().getTable("limelight");
        this.tX = limTable.getEntry("tx");
        this.tS = limTable.getEntry("ts");
        
        this.speedSupplier = speedSupplier;
        m_robotDrive = drive;
        addRequirements(drive);
    }

    double leftX = 0;
    double leftY = 0;
    double rightX = 0;


    double lastTX;
    double lastTS;

    double currTX;
    double currTS;
    @Override
    public void execute() {


        
        leftX = getLeftX.getAsDouble();
        leftY = getLeftY.getAsDouble();
        rightX = getRightX.getAsDouble();
        currTS = time.get()/1000;
        

        
        if(tX.exists()){
            double cL = limTable.getEntry("cl").getDouble(0);
            double tL = limTable.getEntry("tl").getDouble(0);
            currTX = tX.getDouble(0);

            double velAdjustedTX = currTX + ((currTX-lastTX)/(currTS-lastTS)*(cL+tL));
            rightX = velAdjustedTX-LimelightConstants.speakerAimTXFar;
        }

        if(speedSupplier != null) {
            speedControl = speedSupplier.getAsDouble();
        }

        m_robotDrive.drive(
            MathUtil.applyDeadband(-leftY*speedControl, .06),
            MathUtil.applyDeadband(-leftX*speedControl, .06), 
            MathUtil.applyDeadband(-rightX*LimelightConstants.kPRot, .06), fieldCentric, false);
        SmartDashboard.putBoolean("TargetLocked", true);

        if(tX.exists()) {
            lastTS = currTS;
            lastTX = currTX;
        }
    }


    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("TargetLocked", false);

        m_robotDrive.drive(0, 0, 0, false, false);
    }
}
