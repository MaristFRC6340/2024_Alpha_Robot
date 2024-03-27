// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.LEDSubsystem.LEDPattern;
import frc.robot.subsystems.LEDSubsystem.LEDState;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 11;
    public static final int kRearLeftDrivingCanId = 13;
    public static final int kFrontRightDrivingCanId = 15;
    public static final int kRearRightDrivingCanId = 17;

    public static final int kFrontLeftTurningCanId = 10;
    public static final int kRearLeftTurningCanId = 12;
    public static final int kFrontRightTurningCanId = 14;
    public static final int kRearRightTurningCanId = 16;

    public static final boolean kGyroReversed = false;
    public static double kSpeedControl = .4;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 40; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
    public static int kActuatorControllerPort = 1;

    //Deadbands
    public static final double kDriverRTriggerDeadband = .2;
    public static final double kDriverLTriggerDeadband = .2;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }


  public static final class ShooterConstants {
    public static final int kLowerShooterID = 20;
    public static final int kUpperShooterID = 21;
    public static double kDefaultLaunchPower = .7; // WOrks from safe zone
    public static double kDefaultIntakePower = -.1;
    public static double prepareLaunchDelay = 1;
    public static double shootToRestDelay = 1;
  }

  public static final class IntakeConstants {
    public static final int kRollerID = 30;
    public static double kDefaultIntakePower = .3;
    public static double kDefaultOuttakePower = -.7;
    public static double kSlowIntakePower = .2;
    public static double kSlowOuttakePower = -.2;
  }

  public static final class ClimberConstants {

    public static int kLeftClimberID = 40;
    public static int kRightClimberID = 41;
    public static double kP = .01;
    public static double kLowerSpeed = -.9;
    public static double kClimbHeight = 500;

    public static double kLeftMax = -300; // This is negative, note that it is really a "min"
    public static double kRightMax = 300; // This is positive, note it is a true max

    public static double kLeftMin = -10; // Original is zero
    public static double kRightMin = 10; // Original is zero
  }

  public static final class ExtenderConstants {
    public static final int kExtenderID = 50;
    public static double kSourceIntake;
    public static double kAmpOuttake;
    public static double kTrapOuttake;
    public static double kP;

  }

  public static final class JawConstants {
    public static final int kUpperMandibleID = 51;
    public static final int kLowerMandibleID = 52;
    public static double kRotationOffset = .85;
    public static double kIntakePower = .8;
    public static double kOuttakePower = -.5;
  }

  public static final class PneumaticsConstants {
    public static final int kBassForwardChannelPort = 8;
    public static final int kBassReverseChannelPort = 9;
    public static final int kShoulderForwardChannelPort = 10;
    public static final int kShoulderReverseChannelPort = 11;
  }

  public static final class IndexerConstants {

    //TODO: Make sure these CAN ID's are right
    public static final int kIndexerID = 22;
    public static final double kForwardSpeed = 0.4;
    public static final double kBackwardSpeed = -0.4;
    public static final int kPreIndexerID = 23;
    public static double kSlowBackwardsSpeed = -.2;
  }

  public static final class LimelightConstants {
    public static final double kPX = .03;
    public static final double kPY = .03;

    public static final double speakerAimTXClose = 0;

    //JOHN THIS ONE
    //for close drive to
    public static final double speakerAimTYClose = 9;
    public static final double speakerAimTXFar = 0;

    //JOHN THIS ONE
    //for far drive to
    public static final double speakerAimTYFar = -7.7; // Original -3.2, Adjusted for dalton field tolerances


    public static double kTolerance = 1;

    
    //For potential shoot on the move, not currently in use
    public static double kCloseForwardThreshold;
    public static double kCloseTXTolerance;
    public static double kCloseBackwardThreshold;
    public static double kCloseBackwardThreshhold;
    public static double kFarTXTolerance = 4;
    public static double kFarForwardThreshold = -9;
    public static double kFarBackwardThreshold = -10;


    //Used in pneumatics subsystem to determine whether or not the shooter is in range

    //JOHN THIS ONE
    //for close indicators
    public static double kTYInRangeClose = 9;
    public static double kTXInRangeClose = 0; 
    public static double kCloseInRangeTXTolerance = 3;
    public static double kCloseInRangeTYTolerance = 2;

    //JOHN THIS ONE
    //for far indicators
    public static double kTYInRangeFar = -7.7;
    public static double kTXInRangeFar = 0;
    public static double kFarInRangeTYTolerance = 2;
    public static double kFarInRangeTXTolerance = 3;
    public static final double kPRot = .02;
  }

  public static final class BassConstants {
    public static final int kBassID = 31;
    public static double kP = .5;
    public static double kGroundIntakePosition = 62.5;
    public static double kAmpOuttake = 8.5;
    public static double kRestPosition = 5.4;
    public static double kAmpTransferPosition = 13.5;
    public static final double kTransferPose = 34;//was 36
    public static final double minPosition = 0;
  }

  public static final class CelloConstants {
    public static final int kCelloID = 40;
    public static final int kAmpTicklerID = 41;
    public static final double kTravelPosition = -16;//was -18
    public static final double kOuttakePosition = -10;
    public static final double kIntakeTransferPosition = 11.000;
    public static double kOuttakeSpeed = .8;
    public static double kSlowIntakeSpeed = -.3;
  }

  public static final class LEDConstants{
    public static final int stripLength = 161;
    public static final int pwmId = 9;
    public static final int pwmId2 = 8;

    public static LEDPattern blueFlashing(){
        LEDState state = new LEDState(stripLength).fill(new Color(0,0,255));
      return LEDPattern.flashing(state, 100);
    }
    public static LEDPattern redFlashing(){
        LEDState state = new LEDState(stripLength).fill(new Color(255,0,0));
      return LEDPattern.flashing(state, 100);
    }


    //note detected
    public static LEDPattern orange(){
        LEDState state = new LEDState(stripLength).fill(new Color(255, 68, 51));
        return new LEDPattern(0, new LEDState[]{state});
    }
    public static LEDPattern blue(){
      LEDState state = new LEDState(stripLength).fill(new Color(0, 0, 255));
        return new LEDPattern(0, new LEDState[]{state});
    }
    public static LEDPattern red(){
      LEDState state = new LEDState(stripLength).fill(new Color(255, 0, 0));
        return new LEDPattern(0, new LEDState[]{state});
    }
    

    public static LEDPattern ambientManatee(){
        LEDState state = new LEDState(stripLength).fillAlternating(new Color[]{new Color(255,255,255), new Color(99,194,210), new Color(24,75,89), new Color(13,50,63)});
        return LEDPattern.shiftPattern(state, 100);
    }

    public static LEDPattern flashingGreen(){
      LEDState state = new LEDState(stripLength).fill(new Color(0,255,0));
      return LEDPattern.flashing(state, 100);
    }
    public static LEDPattern green(){
      LEDState state = new LEDState(stripLength).fill(new Color(0,255,0));
      return new LEDPattern(0,new LEDState[]{state});
    }

  }
}
