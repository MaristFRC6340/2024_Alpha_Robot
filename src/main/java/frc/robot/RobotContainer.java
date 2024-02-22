// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Commands.HighLaunchNoteCommand;
import frc.robot.Commands.LaunchNoteCommand;
import frc.robot.Commands.LowLaunchNoteCommand;
import frc.robot.Commands.PneumaticsCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  //private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();

  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();

  private final IndexerSubsystem m_IndexerSubsystem = new IndexerSubsystem();

  private final PneumaticsSubsystem m_PneumaticsSubsystem = new PneumaticsSubsystem();

  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // The driver's controller
  static XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  //The actuator's controller
  static XboxController m_actuatorController = new XboxController(OIConstants.kActuatorControllerPort);

  //Create Triggers for bindings

  Trigger driverA = new JoystickButton(m_driverController, XboxController.Button.kA.value);
  Trigger driverB = new JoystickButton(m_driverController, XboxController.Button.kB.value);
  Trigger driverX = new JoystickButton(m_driverController, XboxController.Button.kX.value);
  Trigger driverY = new JoystickButton(m_driverController, XboxController.Button.kY.value);
  Trigger driverLTrigger = new Trigger(() -> m_driverController.getLeftTriggerAxis()>OIConstants.kDriverLTriggerDeadband);
  Trigger driverRTrigger = new Trigger(() -> m_driverController.getRightTriggerAxis()>OIConstants.kDriverRTriggerDeadband);

  Trigger driverLBumper = new JoystickButton(m_actuatorController, XboxController.Button.kLeftBumper.value);
  Trigger driverRBumper = new JoystickButton(m_actuatorController, XboxController.Button.kRightBumper.value);

  Trigger shooterA = new JoystickButton(m_actuatorController, XboxController.Button.kA.value);
  Trigger shooterB = new JoystickButton(m_actuatorController, XboxController.Button.kB.value);
  Trigger shooterX = new JoystickButton(m_actuatorController, XboxController.Button.kX.value);
  Trigger shooterY = new JoystickButton(m_actuatorController, XboxController.Button.kY.value);

  Trigger shooterLBumper = new JoystickButton(m_actuatorController, XboxController.Button.kLeftBumper.value);
  Trigger shooterRBumper = new JoystickButton(m_actuatorController, XboxController.Button.kRightBumper.value);
  SendableChooser<Command> autoChooser;
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    //Register Named Commands
    NamedCommands.registerCommand("HighLaunchNote", new HighLaunchNoteCommand(m_PneumaticsSubsystem, m_ShooterSubsystem, m_IndexerSubsystem));
    NamedCommands.registerCommand("LowLaunchNote", new LowLaunchNoteCommand(m_PneumaticsSubsystem, m_ShooterSubsystem, m_IndexerSubsystem));

    autoChooser = AutoBuilder.buildAutoChooser("default");
    SmartDashboard.putData("Auto Chooser", autoChooser);
     

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // TODO: Uncomment this
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                false, true),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // new JoystickButton(m_driverController, Button.kR1.value)
    //     .whileTrue(new RunCommand(
    //         () -> m_robotDrive.setX(),
    //         m_robotDrive));
    
    // Intake
    driverRTrigger.whileTrue(m_IntakeSubsystem.getSetIntakePowerCommand(() -> m_driverController.getRightTriggerAxis()));
    driverLTrigger.whileTrue(m_IntakeSubsystem.getSetIntakePowerCommand(() -> -1 * m_driverController.getLeftTriggerAxis()));
    driverA.whileTrue(m_IntakeSubsystem.getIntakeCommand());
    driverB.whileTrue(m_IntakeSubsystem.getOuttakeCommand());
    driverX.whileTrue(m_IntakeSubsystem.getSlowIntakeCommand());
    driverY.whileTrue(m_IntakeSubsystem.getSlowOuttakeCommand());

    // Shooter
    shooterA.whileTrue(new SequentialCommandGroup(m_ShooterSubsystem.getPrepareLaunchCommand().withTimeout(2), new LaunchNoteCommand(m_ShooterSubsystem, m_IndexerSubsystem)));

    // Indexer
    shooterB.whileTrue(m_IndexerSubsystem.getRunForwardCommand());
    shooterX.whileTrue(m_IndexerSubsystem.getRunBackwardsCommand());
    
    shooterY.onTrue(m_PneumaticsSubsystem.getToggleTheBassCommand());

    shooterLBumper.whileTrue(m_PneumaticsSubsystem.getRaiseTheBassCommand());
    shooterRBumper.whileTrue(m_PneumaticsSubsystem.getDropTheBassCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    //TODO: Uncomment this
    // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
    //     exampleTrajectory,
    //     m_robotDrive::getPose, // Functional interface to feed supplier
    //     DriveConstants.kDriveKinematics,

    //     // Position controllers
    //     new PIDController(AutoConstants.kPXController, 0, 0),
    //     new PIDController(AutoConstants.kPYController, 0, 0),
    //     thetaController,
    //     m_robotDrive::setModuleStates,
    //     m_robotDrive);

    // // Reset odometry to the starting pose of the trajectory.
    // m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // // Run path following command, then stop at the end.
    // return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
    return null;
  }

  public Command getPneumaticsCommand() {
    return new PneumaticsCommand(m_PneumaticsSubsystem);
  }

  public static XboxController getDriveControlJoystick() {
    return m_driverController;
  }
}
