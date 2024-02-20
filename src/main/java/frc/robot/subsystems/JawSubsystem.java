// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.JawConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class JawSubsystem extends SubsystemBase {

  private CANSparkMax upperMandible;
  private CANSparkMax lowerMandible;
  /** Creates a new ArmSubsystem. */
  public JawSubsystem() {
    upperMandible = new CANSparkMax(JawConstants.kUpperMandibleID, MotorType.kBrushless);
    lowerMandible = new CANSparkMax(JawConstants.kLowerMandibleID, MotorType.kBrushless);

    upperMandible.setSmartCurrentLimit(10);
    lowerMandible.setSmartCurrentLimit(10);

    upperMandible.setIdleMode(IdleMode.kBrake);
    lowerMandible.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //Methods

  public void setUpperMandiblePower(double power) {
      upperMandible.set(power);
  }

  public void setLowerMandiblePower(double power) {
      lowerMandible.set(power);
  }

  public void jawIntake(double power) {
    setUpperMandiblePower(power);
    setLowerMandiblePower(-power);
  }

  public void rotateNote(double power) {
    if(power > 0) {
      setUpperMandiblePower(power);
      setLowerMandiblePower(JawConstants.kRotationOffset*power);
    }
    else {
      setLowerMandiblePower(power);
      setUpperMandiblePower(JawConstants.kRotationOffset*power);
    }
  }

  public void stop() {
    setUpperMandiblePower(0);
    setLowerMandiblePower(0);
  }

  public Command getSetUpperMandibleCommand(double power) {
    return this.startEnd(() -> {
      setUpperMandiblePower(power);
    }, () -> {
      setUpperMandiblePower(0);
    });
  }

  public Command getSetUpperMandibleCommand(DoubleSupplier powerSupplier) {
    return this.startEnd(() -> {
      setUpperMandiblePower(powerSupplier.getAsDouble());
    }, () -> {
      setUpperMandiblePower(0);
    });
  }

  public Command getSetLowerMandibleCommand(double power) {
    return this.startEnd(() -> {
      setLowerMandiblePower(power);
    }, () -> {
      setLowerMandiblePower(0);
    });
  }

  public Command getSetLowerMandibleCommand(DoubleSupplier powerSupplier) {
    return this.startEnd(() -> {
      setLowerMandiblePower(powerSupplier.getAsDouble());
    }, () -> {
      setLowerMandiblePower(0);
    });
  }

  public Command getRotateNoteCommand(double power) {
    return this.startEnd(() -> {
      rotateNote(power);
    }, () -> {
      stop();
    });
  }

  public Command getRotateNoteCommand(DoubleSupplier powerSupplier) {
    return this.startEnd(() -> {
      rotateNote(powerSupplier.getAsDouble());
    }, () -> {
      stop();
    });
  }

  public Command getIntakeNoteCommand() {
    return this.startEnd(() -> {
      jawIntake(JawConstants.kIntakePower);
    }, () -> {
      stop();
    });
  }

  public Command getIntakeNoteCommand(double power) {
    return this.startEnd(() -> {
      jawIntake(power);
    }, () -> {
      stop();
    });
  }

  public Command getIntakeNoteCommand(DoubleSupplier powerSupplier) {
    return this.startEnd(() -> {
      jawIntake(powerSupplier.getAsDouble());
    }, () -> {
      stop();
    });
  }

  public Command getOuttakeNoteCommand() {
    return this.startEnd(() -> {
      jawIntake(JawConstants.kOuttakePower);
    }, () -> {
      stop();
    });
  }
}
