package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CelloConstants;

//Called the cello because its a smaller version of the bass
//Ampersand pivot   
public class CelloSubsystem extends SubsystemBase{
    
    private CANSparkMax celloMotor;
    private RelativeEncoder relativeEncoder;
    private SparkPIDController pidController;

    public CelloSubsystem() {
        celloMotor = new CANSparkMax(CelloConstants.kCelloID, MotorType.kBrushless);

        celloMotor.setSmartCurrentLimit(40);

        celloMotor.setIdleMode(IdleMode.kBrake);

        relativeEncoder = celloMotor.getEncoder();

        
        pidController = celloMotor.getPIDController();

        



        pidController.setP(.03);
        pidController.setI(0);
        pidController.setD(0);

        pidController.setReference(0, ControlType.kPosition);
    }

    public void setPower(double power) {
        celloMotor.set(power);
    }

    public void setPosition(double position) {
        pidController.setReference(position, ControlType.kPosition);
    }

    public Command getSetPowerCommand(double power) {
        return this.startEnd(() -> {
            celloMotor.set(power);
        }, () -> {
            setPower(0);
        });
    }

    public Command getSetPowerCommand(DoubleSupplier powSupplier) {
        return this.runEnd(() -> {
            celloMotor.set(powSupplier.getAsDouble());
        }, () -> {
            setPower(0);
        });
    }

    public Command getSetPositionCommand(double position) {
        return this.runOnce(() -> {
            this.setPosition(
                position
            );
        });
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Cello Position", relativeEncoder.getPosition());
    }


}
