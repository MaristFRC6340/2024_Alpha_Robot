package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BassConstants;
public class TheBassSubsystem extends SubsystemBase {
    private CANSparkMax theBassMotor;

    private RelativeEncoder bassEncoder;

    private SparkPIDController bassPID;
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
        return this.startEnd(() -> {
            setBasePower(powerSupplier.getAsDouble());
        }, () -> {
            stop();
        });
    }

    public Command getGoToPositionCommand(double position) {
        return this.runOnce(() -> {
            goToPosition(position);
        });
    }

    public Command getGoToPositionCommand(DoubleSupplier positionSupplier) {
        return this.runEnd(() -> {
            goToPosition(positionSupplier.getAsDouble());
        }, () -> {
            stop();
        });
    }

    public Command getDropTheBassCommand() {
        return this.runOnce(() -> {
            dropTheBass();
        });
    }

    public Command getGoToAmpOuttakeCommand() {
        return this.runOnce(() -> {
            goToAmpOuttake();
        });
    }

    public Command getHoldPositionCommand() {
        return this.run(() -> {
            goToPosition(getPosition());
        });
    }
}
