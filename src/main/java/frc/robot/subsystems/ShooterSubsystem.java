// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  private CANSparkMax lowerShooter;
  private CANSparkMax upperShooter;
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    lowerShooter = new CANSparkMax(ShooterConstants.kLowerShooterID, MotorType.kBrushless);
    upperShooter = new CANSparkMax(ShooterConstants.kUpperShooterID, MotorType.kBrushless);

    lowerShooter.setSmartCurrentLimit(40);
    upperShooter.setSmartCurrentLimit(40);

    lowerShooter.setIdleMode(IdleMode.kCoast);
    upperShooter.setIdleMode(IdleMode.kCoast);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Vel", lowerShooter.getEncoder().getVelocity());
    SmartDashboard.putBoolean("ShooterSpun", spunUp());
  }

  //Methods
  public boolean spunUp(){
    return lowerShooter.getEncoder().getVelocity() > 3800;
  }
  /**
   * Sets the shooter to a desired power
   * @param power
   */
  public void setShooterPower(double power) {
    lowerShooter.set(power);
    upperShooter.set(power);
  }

  /**
   * Stops the shooter motors
   */
  public void stop() {
    lowerShooter.set(0);
    upperShooter.set(0);
  }

  /**
   * Sets the shooter motors to the launch speed
   */
  public void prepareLaunch() {
    setShooterPower(ShooterConstants.kDefaultLaunchPower);
  }

  /**
   * Sets the shooter motors to the default speed for intaking
   */
  public void intakeFromSource() {
    setShooterPower(ShooterConstants.kDefaultIntakePower);
  }

  //Command Factory

  /**
   * Command to set the shooter power to a given double
   * @param power
   * @return command to set shooter power
   */
  public Command getSetShooterPowerCommand(double power) {
    return this.startEnd(() -> {
      setShooterPower(power);
    }, () -> {
      stop();
    });
  }

/**
 * Command to set the shooter power to a given changing powerSupplier
 * @param powerSupplier
 * @return stopped shooter
 */
  public Command getSetShooterPowerCommand(DoubleSupplier powerSupplier) {
    //System.out.println("set to " + powerSupplier.getAsDouble());
    return this.runEnd(() -> {
      setShooterPower(powerSupplier.getAsDouble());
      System.out.println("set to " + powerSupplier.getAsDouble());
    }, () -> {
      stop();
    });
  }

  /**
   * the command to stop the shooter
   * @return stopped shooter
   */
  public Command getStopShooterCommand() {
    return this.runOnce(() -> {
      stop();
    });
  }

  /**
   * Sets the shooter power to the default intake power
   * @return stopped shooter
   */
  public Command getIntakeSourceCommand() {
    return this.startEnd(() -> {
      intakeFromSource();
    }, () -> {
      stop();
    });
  }

  /**
   * Spins up the shooter to the default launch speed
   * @return spinning shooter
   */
  public Command getPrepareLaunchCommand() {
    return this.runOnce(() -> {
      prepareLaunch();
    });
  }

  /**
   * Spins up the shooter to a given launch speed
   * @param power
   * @return spinning shooter
   */
  public Command getPrepareLaunchCommand(double power) {
    return this.runOnce(() -> {
      setShooterPower(power);
    });
  }

  /**
   * Spins up the shooter to a given changing power
   * @param powSupplier
   * @return spinning shooter
   */
  public Command getPrepareLaunchCommand(DoubleSupplier powSupplier) {
    return this.run(() -> {
      setShooterPower(powSupplier.getAsDouble());
    });
  }
}
