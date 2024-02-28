package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.BassConstants;
public class TheBassSubsystem extends SubsystemBase {
    private CANSparkMax theBassMotor;

    private RelativeEncoder bassEncoder;

    private SparkPIDController bassPID;

    private double currentReference;


    public TheBassSubsystem() {
        theBassMotor = new CANSparkMax(BassConstants.kBassID, MotorType.kBrushless);

        theBassMotor.setSmartCurrentLimit(40);

        theBassMotor.setIdleMode(IdleMode.kBrake);

        bassEncoder = theBassMotor.getEncoder();

        bassPID = theBassMotor.getPIDController();

        bassPID.setP(BassConstants.kP);
    }

    public void setBasePower(double power) {
        theBassMotor.set(power);
    }

    public void goToPosition(double position) {
        bassPID.setReference(position, ControlType.kPosition);
    }

    public double getPosition() {
        return bassEncoder.getPosition();
    }

    public void dropTheBass() {
        goToPosition(BassConstants.kGroundIntakePosition);
    }

    public void goToTransfer() {
        goToPosition(BassConstants.kTransferPose);
    }

    public void stop() {
        setBasePower(0);
    }
    public void goToAmpOuttake() {
        goToPosition(BassConstants.kAmpOuttake);
    }

    public Command getSetPowerCommand(double power) {
        return this.startEnd(() -> {
            setBasePower(power);
        }, () -> {
            stop();
        });
    }

    public Command getSetPowerCommand(DoubleSupplier powerSupplier) {
        return this.runEnd(() -> {
            setBasePower(powerSupplier.getAsDouble());
        }, () -> {
            stop();
        });
    }

    public Command getGoToPositionCommand(double position) {
        return this.startEnd(() -> {
            goToPosition(position);
        }, () -> {
            stop();
        })
        .until(() -> this.isAt(position));
    }

    public Command getGoToPositionCommand(DoubleSupplier positionSupplier) {
        return this.runEnd(() -> {
            goToPosition(positionSupplier.getAsDouble());
        }, () -> {
            stop();
        }).until(() -> this.isAt(positionSupplier.getAsDouble()));
    }

    public Command getDropTheBassCommand() {
        return this.startEnd(() -> {
            dropTheBass();
            System.out.println("Dropped the Base");
        }, () -> {
            stop();
        }).until(() -> this.isAt(BassConstants.kGroundIntakePosition));
    }

    public Command getGoToAmpOuttakeCommand() {
        return this.startEnd(() -> {
            goToAmpOuttake();
        }, () -> {
            stop();
        }).until(() -> this.isAt(BassConstants.kAmpOuttake));
    }

    public Command getGoToTransferCommand() {
        return this.startEnd(() -> {
            goToTransfer();
        }, () -> {
            stop();
        }).until(() -> this.isAt(BassConstants.kTransferPose));
    }

    public Command getHoldPositionCommand() {
        return this.run(() -> {
            goToPosition(getPosition());
        });
    }

    public Command getHoldPositionCommand(DoubleSupplier hold) {
        return this.runOnce(() -> {
            goToPosition(hold.getAsDouble());
        });
    }

    public boolean isAt(double position) {
        double current = bassEncoder.getPosition();

        return Math.abs(current-position)<3.5;
    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("Bass Position", bassEncoder.getPosition());

        SmartDashboard.putNumber("POV", RobotContainer.getActuatorController().getPOV());
    }
}
