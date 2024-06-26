package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.DriveSubsystem;

public class PIDDriveTargetLockCommand extends Command{
    private final DriveSubsystem m_robotDrive;

    private double speedControl=DriveConstants.kSpeedControl;


    DoubleSupplier getLeftX;
    DoubleSupplier getLeftY;
    DoubleSupplier getRightX;
    NetworkTableEntry tX;
    NetworkTable limTable;


    double turnPower = 0;

    private boolean fieldCentric = true;

    PIDController targetingPID = new PIDController(.02, 0, .0000001);

    private DoubleSupplier speedSupplier;
    public PIDDriveTargetLockCommand(DriveSubsystem drive, DoubleSupplier getLeftX, DoubleSupplier getLeftY, DoubleSupplier getRightX) {
        this.getLeftX = getLeftX;
        this.getLeftY = getLeftY;
        this.getRightX = getRightX;
        
        this.limTable = NetworkTableInstance.getDefault().getTable("limelight");
        this.tX = limTable.getEntry("ty");
        m_robotDrive = drive;
        addRequirements(drive);

    }

    public PIDDriveTargetLockCommand(DriveSubsystem drive, DoubleSupplier getLeftX, DoubleSupplier getLeftY, DoubleSupplier getRightX, double speedControl) {
        this.getLeftX = getLeftX;
        this.getLeftY = getLeftY;
        this.getRightX = getRightX;

        this.limTable = NetworkTableInstance.getDefault().getTable("limelight");
        this.tX = limTable.getEntry("ty");
        
        this.speedControl = speedControl;
        m_robotDrive = drive;
        addRequirements(drive);
    }

    public PIDDriveTargetLockCommand(DriveSubsystem drive, DoubleSupplier getLeftX, DoubleSupplier getLeftY, DoubleSupplier getRightX, DoubleSupplier speedSupplier) {
        this.getLeftX = getLeftX;
        this.getLeftY = getLeftY;
        this.getRightX = getRightX;

        this.limTable = NetworkTableInstance.getDefault().getTable("limelight");
        this.tX = limTable.getEntry("ty");
        
        this.speedSupplier = speedSupplier;
        m_robotDrive = drive;
        addRequirements(drive);
    }

    double leftX = 0;
    double leftY = 0;
    double rightX = 0;

    @Override
    public void execute() {

        leftX = getLeftX.getAsDouble();
        leftY = getLeftY.getAsDouble();
        rightX = getRightX.getAsDouble();
        
        if(tX.exists()){
            rightX = limTable.getEntry("tx").getDouble(0)-LimelightConstants.speakerAimTXFar;
        }


        double rotSpeed = targetingPID.calculate(LimelightHelpers.getTX("limelight"), LimelightConstants.speakerAimTXFar);
        if(speedSupplier != null) {
            speedControl = speedSupplier.getAsDouble();
        }

        if(LimelightHelpers.getTV("limelight")) {
        m_robotDrive.drive(
            MathUtil.applyDeadband(-leftY*speedControl, .06),
            MathUtil.applyDeadband(-leftX*speedControl, .06), 
            MathUtil.applyDeadband(rotSpeed, .06), fieldCentric, false);
        }
        else {
            m_robotDrive.drive(
            MathUtil.applyDeadband(-leftY*speedControl, .06),
            MathUtil.applyDeadband(-leftX*speedControl, .06), 
            MathUtil.applyDeadband(-rightX, .06), fieldCentric, false);
        }

        SmartDashboard.putBoolean("TargetLocked", true);

    }


    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("TargetLocked", false);

        m_robotDrive.drive(0, 0, 0, false, false);
    }
}
