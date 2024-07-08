package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;

public class PIDAimCommand extends Command{
    private DriveSubsystem m_DriveSubsystem;

    public PIDAimCommand(DriveSubsystem drive) {
        this.m_DriveSubsystem=drive;
        addRequirements(drive);
    }


    PIDController turningPID = new PIDController(.01, 0, 0);

    @Override
    public void execute() {
        double xError = LimelightHelpers.getTX("limelight");
        m_DriveSubsystem.drive(0, 0, turningPID.calculate(xError, 0), false, false);
    }

    @Override
    public void end(boolean interrupted) {
        m_DriveSubsystem.drive(0, 0, 0, false, false);
    }

}
