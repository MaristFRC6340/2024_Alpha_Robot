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
import frc.robot.Constants.CelloConstants;

public class AmpTicklerSubsystem extends SubsystemBase{

    private CANSparkMax ampTickler;
    private SparkPIDController pidController;
    private RelativeEncoder relativeEncoder;


    public AmpTicklerSubsystem() {
        this.ampTickler = new CANSparkMax(CelloConstants.kAmpTicklerID, MotorType.kBrushless);

        ampTickler.setSmartCurrentLimit(20);
        ampTickler.setIdleMode(IdleMode.kBrake);

        pidController = ampTickler.getPIDController();

        relativeEncoder = ampTickler.getEncoder();
    }

    public void setSpeed(double speed) {
        ampTickler.set(speed);
    }

    public void rollCounts(double counts) {
        pidController.setReference(relativeEncoder.getPosition() + counts, ControlType.kPosition);
    }

    public Command getSetSpeedCommand(double speed) {
        return this.startEnd(() -> {
            this.setSpeed(speed);
        }, () -> {
            this.setSpeed(0);
        });
    }

    public Command getSetSpeedCommand(DoubleSupplier powerSupplier) {
        return this.runEnd(() -> {
            this.setSpeed(powerSupplier.getAsDouble());
        }, () -> {
            this.setSpeed(0);
        });
    }

    public Command getRollCountsCommand(double counts) {
        return this.runOnce(() -> rollCounts(counts));
    }

    public Command getStopCommand() {
        return this.runOnce(() -> setSpeed(0));
    }
}