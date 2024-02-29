// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  private CANSparkMax leftClimber;
  private CANSparkMax rightClimber;

  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;

  private SparkPIDController leftClimberPID;
  private SparkPIDController rightClimberPID;

  private boolean climberUp = false;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    leftClimber = new CANSparkMax(ClimberConstants.kLeftClimberID, MotorType.kBrushless);
    rightClimber = new CANSparkMax(ClimberConstants.kRightClimberID, MotorType.kBrushless);

    leftClimber.setIdleMode(IdleMode.kBrake);
    rightClimber.setIdleMode(IdleMode.kBrake);

    leftClimber.setSmartCurrentLimit(40);
    rightClimber.setSmartCurrentLimit(40);

    leftEncoder = leftClimber.getEncoder();
    rightEncoder = rightClimber.getEncoder();

    leftClimberPID = leftClimber.getPIDController();
    rightClimberPID = rightClimber.getPIDController();

    leftClimberPID.setP(ClimberConstants.kP);
    rightClimberPID.setP(ClimberConstants.kP);

    SmartDashboard.putBoolean("RESET LEFT CLIMBER ENCODER", false);
    SmartDashboard.putBoolean("RESET RIGHT CLIMBER ENCODER", false);


  }

  @Override
  public void periodic() {
    if(rightEncoder.getPosition() > ClimberConstants.kClimbHeight/2.0){
      climberUp = true;
    }
    else{
      climberUp = false;
    }
    SmartDashboard.putNumber("RIGHT CLIMBER ENCODER", rightEncoder.getPosition());
    SmartDashboard.putNumber("LEFT CLIMBER ENCODER", leftEncoder.getPosition());

    if(SmartDashboard.getBoolean("RESET LEFT CLIMBER ENCODER", false)){
      SmartDashboard.putBoolean("RESET LEFT CLIMBER ENCODER", false);
      leftEncoder.setPosition(0);
    }
    if(SmartDashboard.getBoolean("RESET RIGHT CLIMBER ENCODER", false)){
      SmartDashboard.putBoolean("RESET RIGHT CLIMBER ENCODER", false);
      rightEncoder.setPosition(0);
    }

  }




  //Methods

  /**
   * Sets the climber to a given power
   * @param power
   */
  public void setClimberPower(double power) {
    leftClimber.set(-power);
    rightClimber.set(power);
    SmartDashboard.putNumber("RIGHT CLIMBER", this.getRightEncoderCounts());
    SmartDashboard.putNumber("LEFT CLIMBER", this.getLeftEncoderCounts());

    System.out.println("Left Encoder: " + this.getLeftEncoderCounts());
    System.out.println("Right Encoder: " + this.getRightEncoderCounts());
  }

  /**
   * Sets the PID to a given encoder position
   * @param encoderCounts
   */
  public void goToPosition(double encoderCounts) {
    if(encoderCounts > 0)encoderCounts = 0;
    
    leftClimberPID.setReference(encoderCounts, CANSparkMax.ControlType.kPosition);
    rightClimberPID.setReference(encoderCounts, CANSparkMax.ControlType.kPosition);
  }

  /**
   * Returns the encoder counts of the left encoder
   * @return
   */
  public double getLeftEncoderCounts() {
    return leftEncoder.getPosition();
  }

  /**
   * Returns the encoder counts of the right encoder
   * @return
   */
  public double getRightEncoderCounts() {
    return rightEncoder.getPosition();
  }

  /**
   * Raises the climber to the climbing height
   */
  public void goToClimbHeight() {
    goToPosition(ClimberConstants.kClimbHeight);
  }

  public void toggleClimber(){
    if(climberUp){
      //goToPosition(0);
      System.out.println("GO DOWN");
    }
    else{
      //goToClimbHeight();
      System.out.println("GO UP");
    }
  }

  /**
   * Stops the climber
   */
  public void stop() {
    setClimberPower(0);
  }

  /**
   * Command to set the climber to a given power
   * @param power
   * @return stopped climber
   */
  public Command getSetClimberPowerCommand(double power) {
    return this.startEnd(() -> {
      setClimberPower(power);
    }, () -> {
      stop();
    });
  }

  /**
   * Command to set the climber to a changing given power
   * @param power
   * @return stopped climber
   */
  public Command getSetClimberPowerCommand(DoubleSupplier powerSupplier) {
    return this.startEnd(() -> {
      setClimberPower(powerSupplier.getAsDouble());
    }, () -> {
      stop();
    });
  }

  public Command setRightClimberPower(DoubleSupplier powerSupplier){
    return this.startEnd(()->{rightClimber.set(powerSupplier.getAsDouble());}, ()->{rightClimber.set(0);});
  }
  public Command setLeftClimberPower(DoubleSupplier powerSupplier){
    return this.startEnd(()->{leftClimber.set(powerSupplier.getAsDouble());}, ()->{leftClimber.set(0);});
  }

  /**
   * Raises the hooks
   * @return stopped climber
   */
  public Command getRaiseHooksCommand() {
    return this.startEnd(() -> {
      goToClimbHeight();
    }, () -> {
      stop();
    });
  }
  
  /**
   * Starts lowering the hooks
   * @return lowering climber
   */
  public Command getLowerHooksCommand() {
    return this.startEnd(() -> {
      setClimberPower(ClimberConstants.kLowerSpeed);
    }, () -> {
      //none
    });
  }

  /**
   * Holds the position in place
   * Default Command
   * @return
   */
  public Command getHoldPositionCommand() {
    return this.run(() -> {
      goToPosition(getLeftEncoderCounts());
    });
  }

  


}
