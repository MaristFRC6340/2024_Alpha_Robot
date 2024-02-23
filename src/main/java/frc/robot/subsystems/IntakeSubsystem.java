// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private CANSparkMax roller;
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    roller = new CANSparkMax(IntakeConstants.kRollerID, MotorType.kBrushless);

    roller.setSmartCurrentLimit(40);

    roller.setIdleMode(IdleMode.kBrake);
  }

  public IntakeSubsystem(IndexerSubsystem m_IndexerSubsystem) {
    //TODO Auto-generated constructor stub
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //Methods

  /**
   * Sets the intake power to a given power
   * @param power
   */
  public void setIntakePower(double power) {
    roller.set(power);
  }

  /**
   * Sets the roller power to the default intaking speed
   */
  public void intake() {
    setIntakePower(IntakeConstants.kDefaultIntakePower);
  }

  /**
   * Stops the roller
   */
  public void stop() {
    setIntakePower(0);
  }

  /**
   * Sets the roller to the default outtake power
   */
  public void outtake() {
    setIntakePower(IntakeConstants.kDefaultOuttakePower);
  }

  /**
   * Sets the roller to a slower intaking speed
   */
  public void slowIntake() {
    setIntakePower(IntakeConstants.kSlowIntakePower);
  }

  /**
   * Sets the roller to a slower outtaking speed
   */
  public void slowOuttake() {
    setIntakePower(IntakeConstants.kSlowOuttakePower);
  }

  //Command Factories

  /**
   * Sets the intake to a constant power
   * @param power
   * @return stopped intake
   */
  public Command getSetIntakePowerCommand(double power) {
    return this.startEnd(() -> {
      setIntakePower(power);
    }, () -> {
      stop();
    });
  }

  /**
   * Sets the intake to a constant power
   * @param power
   * @return stopped intake
   */
  public Command getSetIntakePowerCommand(DoubleSupplier powerSupplier) {
    System.out.println("set to " + powerSupplier.getAsDouble());
    return this.runEnd(() -> {
      setIntakePower(powerSupplier.getAsDouble());
    }, () -> {
      stop();
    });
  }

  /**
   * Sets the intake to the default power for intaking
   * @return stopped rollers
   */
  public Command getIntakeCommand() {
    return this.startEnd(() -> {
      intake();
    }, () -> {
      stop();
    });
  }

  /**
   * Sets the intake to the default power for outtaking
   * @return stopped rollers
   */
  public Command getOuttakeCommand() {
    return this.startEnd(() -> {
      outtake();
    }, () -> {
      stop();
    });
  }

  /**
   * Sets the intake to the slow intaking power
   * @return stopped shooter
   */
  public Command getSlowIntakeCommand() {
    return this.startEnd(() -> {
      slowIntake();
    }, () -> {
      stop();
    });
  }

  /**
   * Sets the intake to the slow outtaking power
   * @return stopped shooter
   */
  public Command getSlowOuttakeCommand() {
    return this.startEnd(() -> {
      slowOuttake();
    }, () -> {
      stop();
    });
  }
}
