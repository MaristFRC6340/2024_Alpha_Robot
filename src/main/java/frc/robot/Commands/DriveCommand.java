package frc.robot.Commands;

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

    private double kP = .025;

    double turnPower = 0;

    private boolean fieldCentric = true;

    public DriveCommand(DriveSubsystem drive) {
        m_robotDrive = drive;
        addRequirements(drive);
    }

    double leftX = 0;
    double leftY = 0;
    double rightX = 0;

    @Override
    public void execute() {

        leftX = Robot.getDriveControllerJoystick().getRawAxis(0);
        leftY = Robot.getDriveControllerJoystick().getRawAxis(1);
        rightX = Robot.getDriveControllerJoystick().getRawAxis(4);

        m_robotDrive.drive(
            MathUtil.applyDeadband(-leftY*speedControl, .06),
            MathUtil.applyDeadband(-leftX*speedControl, .06), 
            MathUtil.applyDeadband(-rightX*speedControl, .06), fieldCentric, true);

            
    }
}
