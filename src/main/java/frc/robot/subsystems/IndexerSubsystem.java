package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {
    private CANSparkMax indexerMotor;

    public IndexerSubsystem() {
        indexerMotor = new CANSparkMax(IndexerConstants.kIndexerID, MotorType.kBrushed);
        indexerMotor.setSmartCurrentLimit(40);
        indexerMotor.setIdleMode(IdleMode.kBrake);
    }

    /**
     * 
     */
    public void runForwards() {
        indexerMotor.set(IndexerConstants.kForwardSpeed);
    }
    public void runBackwards() {
        indexerMotor.set(IndexerConstants.kBackwardSpeed);
    }
    public void stop() {
        indexerMotor.set(0);
    }
    public void setPower(double speed) {
        indexerMotor.set(speed);
    }

    public Command getSetPowerCommand(double power) {
        return this.startEnd(()->{
            this.setPower(power);
        }, ()->{
            this.setPower(0);
        });
    }

    public Command getSetPowerCommand(DoubleSupplier power) {
        return this.runEnd(()->{
            this.setPower(power.getAsDouble());
        }, ()->{
            this.setPower(0);
        });
    }

    public Command getStopCommand() {
        return this.runOnce(()->{
            this.setPower(0);
        });
    }

    public Command getRunForwardCommand() {
        return this.startEnd(()->{
            this.runForwards();
        }, ()->{
            this.stop();
        });
    }

    public Command getRunBackwardsCommand() {
        return this.startEnd(()->{
            this.runBackwards();
        }, ()->{
            this.stop();
        });
    }

}
