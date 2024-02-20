// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.ExtenderConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExtenderSubsystem extends SubsystemBase {

  private CANSparkMax armExtender;

  private RelativeEncoder encoder;

  private SparkPIDController extenderPID;
  /** Creates a new ArmSubsystem. */
  public ExtenderSubsystem() {
    armExtender = new CANSparkMax(ExtenderConstants.kExtenderID, MotorType.kBrushless);

    armExtender.setSmartCurrentLimit(40);

    armExtender.setIdleMode(IdleMode.kBrake);

    encoder = armExtender.getEncoder();

    extenderPID = armExtender.getPIDController();
    extenderPID.setP(ExtenderConstants.kP);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  //Methods

  public void setExtenderPower(double power) {
    armExtender.set(power);
  }

  public void stop() {
    armExtender.set(0);
  }

  public void goToPosition()
}
