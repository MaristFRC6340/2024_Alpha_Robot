package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends Command{
    private final DriveSubsystem m_robotDrive;

    private double speedControl=DriveConstants.kSpeedControl;
    private double rateLimit = 1;

    private SlewRateLimiter filterX = new SlewRateLimiter(rateLimit);
    private SlewRateLimiter filterY = new SlewRateLimiter(rateLimit);

    DoubleSupplier getLeftX;
    DoubleSupplier getLeftY;
    DoubleSupplier getRightX;


    DoubleSupplier speedSupplier;
    private double kP = .025;

    double turnPower = 0;

    private boolean fieldCentric = true;

    public DriveCommand(DriveSubsystem drive, DoubleSupplier getLeftX, DoubleSupplier getLeftY, DoubleSupplier getRightX) {
        this.getLeftX = getLeftX;
        this.getLeftY = getLeftY;
        this.getRightX = getRightX;

        m_robotDrive = drive;
        addRequirements(drive);
    }

    public DriveCommand(DriveSubsystem drive, DoubleSupplier getLeftX, DoubleSupplier getLeftY, DoubleSupplier getRightX, double speedControl) {
        this.getLeftX = getLeftX;
        this.getLeftY = getLeftY;
        this.getRightX = getRightX;
        
        this.speedControl = speedControl;
        m_robotDrive = drive;
        addRequirements(drive);
    }

    public DriveCommand(DriveSubsystem drive, DoubleSupplier getLeftX, DoubleSupplier getLeftY, DoubleSupplier getRightX, DoubleSupplier speedControlSupplier) {
        this.getLeftX = getLeftX;
        this.getLeftY = getLeftY;
        this.getRightX = getRightX;
        
        this.speedSupplier = speedControlSupplier;
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

        if(speedSupplier!=null) {
            speedControl = speedSupplier.getAsDouble();
        }

        m_robotDrive.drive(
            MathUtil.applyDeadband(-leftY*speedControl, .06),
            MathUtil.applyDeadband(-leftX*speedControl, .06), 
            MathUtil.applyDeadband(-rightX*speedControl, .06), fieldCentric, false);
    }


    @Override
    public void end(boolean interrupted) {
        m_robotDrive.drive(0, 0, 0, false, false);
    }
}
