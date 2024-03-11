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
import frc.robot.Commands.AimAndDriveFarCommand;
import frc.robot.Commands.DriveCommand;
import frc.robot.Commands.DriveToCloseShotCommand;
import frc.robot.Commands.DriveToFarShotCommand;
import frc.robot.Commands.DriveToSourceCommand;
import frc.robot.Commands.HighLaunchNoteCommand;
import frc.robot.Commands.LaunchNoteCommand;
import frc.robot.Commands.LowLaunchNoteCommand;
import frc.robot.Commands.OrthagonalizeCommand;
import frc.robot.Commands.PointToAprilTagCommand;
import frc.robot.Commands.TransferToIndexerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.OIConstants;
//import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.JawSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TheBassSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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

  //private final ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();

  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  private final TheBassSubsystem m_TheBassSubsystem = new TheBassSubsystem();

  //private final JawSubsystem m_JawSubsystem = new JawSubsystem();
  // The driver's controller
  static XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  //The actuator's controller
  static XboxController m_actuatorController = new XboxController(OIConstants.kActuatorControllerPort);

  //Create Triggers for bindings
  Trigger driverStart = new JoystickButton(m_driverController, XboxController.Button.kStart.value);
  Trigger driverA = new JoystickButton(m_driverController, XboxController.Button.kA.value);
  Trigger driverB = new JoystickButton(m_driverController, XboxController.Button.kB.value);
  Trigger driverX = new JoystickButton(m_driverController, XboxController.Button.kX.value);
  Trigger driverY = new JoystickButton(m_driverController, XboxController.Button.kY.value);
  Trigger driverLTrigger = new Trigger(() -> m_driverController.getLeftTriggerAxis()>OIConstants.kDriverLTriggerDeadband);
  Trigger driverRTrigger = new Trigger(() -> m_driverController.getRightTriggerAxis()>OIConstants.kDriverRTriggerDeadband);
  Trigger driverLBumper = new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value);
  Trigger driverRBumper = new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value);
  Trigger driverLeftStickButton = new JoystickButton(m_driverController, XboxController.Button.kLeftStick.value);
  Trigger actuatorA = new JoystickButton(m_actuatorController, XboxController.Button.kA.value);
  Trigger actuatorB = new JoystickButton(m_actuatorController, XboxController.Button.kB.value);
  Trigger actuatorX = new JoystickButton(m_actuatorController, XboxController.Button.kX.value);
  Trigger actuatorY = new JoystickButton(m_actuatorController, XboxController.Button.kY.value);
  Trigger actuatorLBumper = new JoystickButton(m_actuatorController, XboxController.Button.kLeftBumper.value);
  Trigger actuatorRBumper = new JoystickButton(m_actuatorController, XboxController.Button.kRightBumper.value);

  Trigger actuatorDpadUp = new Trigger(() -> m_actuatorController.getPOV()==0);
  Trigger actuatorDpadLeft = new Trigger(() -> m_actuatorController.getPOV()==270);
  Trigger actuatorDpadRight = new Trigger(() -> m_actuatorController.getPOV()==90);
  Trigger actuatorDpadDown = new Trigger(() -> m_actuatorController.getPOV()==180);

  Trigger actuatorLeftY = new Trigger(() -> Math.abs(m_actuatorController.getLeftY()) > .1);
  Trigger actuatorLeftX = new Trigger(() -> Math.abs(m_actuatorController.getLeftX()) > .1);
  Trigger actuatorRightY = new Trigger(() -> Math.abs(m_actuatorController.getRightY()) > .1);
  Trigger actuatorRightX = new Trigger(() -> Math.abs(m_actuatorController.getRightX()) > .1);


  Trigger actuatorLTrigger = new Trigger(() -> m_actuatorController.getLeftTriggerAxis()>OIConstants.kDriverLTriggerDeadband);
  Trigger actuatorRTrigger = new Trigger(() -> m_actuatorController.getRightTriggerAxis()>OIConstants.kDriverRTriggerDeadband);
  Trigger driverDpad = new Trigger(()->m_driverController.getPOV()!=-1);

  Trigger inShootingRange = new Trigger(()->m_robotDrive.inShootingRange(m_PneumaticsSubsystem.getShoulderRaised()));

  SendableChooser<Command> autoChooser;
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    //Register Named Commands
    NamedCommands.registerCommand("HighLaunchNoteKeepShooterRunning", new HighLaunchNoteCommand(m_PneumaticsSubsystem, m_ShooterSubsystem, m_IndexerSubsystem, true));

    NamedCommands.registerCommand("HighLaunchNote", new HighLaunchNoteCommand(m_PneumaticsSubsystem, m_ShooterSubsystem, m_IndexerSubsystem));
    NamedCommands.registerCommand("LowLaunchNote", new LowLaunchNoteCommand(m_PneumaticsSubsystem, m_ShooterSubsystem, m_IndexerSubsystem));
    NamedCommands.registerCommand("StartShooter", m_ShooterSubsystem.getPrepareLaunchCommand());
    NamedCommands.registerCommand("StopShooter", m_ShooterSubsystem.getStopShooterCommand());
    NamedCommands.registerCommand("ForwardIndexer", m_IndexerSubsystem.getRunForwardCommand());
    NamedCommands.registerCommand("StopIndexer", m_IndexerSubsystem.getStopCommand());

    NamedCommands.registerCommand("LaunchNote", new LaunchNoteCommand(m_ShooterSubsystem, m_IndexerSubsystem).withTimeout(.4));
    //NamedCommands.registerCommand("StartGroundIntake", new ParallelCommandGroup(m_IndexerSubsystem.getGroundIntakeCommand(), m_IntakeSubsystem.getIntakeCommand()));
    NamedCommands.registerCommand("BackwardsIndexer", m_IndexerSubsystem.getRunBackwardsCommand());
    NamedCommands.registerCommand("RunIntake", m_IntakeSubsystem.getIntakeCommand());
    NamedCommands.registerCommand("RunOuttake", m_IntakeSubsystem.getOuttakeCommand());
    NamedCommands.registerCommand("RaiseTheBass", m_TheBassSubsystem.getGoToAmpOuttakeCommand().withTimeout(.5));
    NamedCommands.registerCommand("TransferTheBass", m_TheBassSubsystem.getGoToTransferCommand().withTimeout(.5));
    NamedCommands.registerCommand("DropTheBass", m_TheBassSubsystem.getDropTheBassCommand().withTimeout(1));
    NamedCommands.registerCommand("RaiseShoulder", m_PneumaticsSubsystem.getRaiseShoulderCommand().withTimeout(.5));
    NamedCommands.registerCommand("DropShoulder", m_PneumaticsSubsystem.getDropShoulderCommand().withTimeout(.5));

    NamedCommands.registerCommand("StartIntake", m_IntakeSubsystem.getStartIntakeCommand());
    NamedCommands.registerCommand("StartIntakeSlow", m_IntakeSubsystem.getStartSlowIntakeCommand());
    NamedCommands.registerCommand("StopIntake", m_IntakeSubsystem.getStopIntakeCommand());

    // NamedCommands.registerCommand("IntakeAndShootLow", new IntakeAndShootLowCommand(m_IndexerSubsystem, m_ShooterSubsystem, m_PneumaticsSubsystem));
    // NamedCommands.registerCommand("IntakeAndShootHigh", new IntakeAndShootHighCommand(m_IndexerSubsystem, m_ShooterSubsystem, m_PneumaticsSubsystem));

    NamedCommands.registerCommand("TransferNote", new SequentialCommandGroup(m_TheBassSubsystem.getGoToTransferCommand(), new TransferToIndexerCommand(m_IndexerSubsystem, m_IntakeSubsystem).withTimeout(1.5), m_TheBassSubsystem.getDropTheBassCommand()));

    autoChooser = AutoBuilder.buildAutoChooser("default");
    SmartDashboard.putData("Auto Chooser", autoChooser);
     

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // m_robotDrive.setDefaultCommand(
    //     // The left stick controls translation of the robot.
    //     // Turning is controlled by the X axis of the right stick.
    //     new RunCommand(
    //         () -> m_robotDrive.drive(
    //             -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
    //             -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
    //             -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
    //             false, true),
    //         m_robotDrive));
    
    m_robotDrive.setDefaultCommand(
      new DriveCommand(m_robotDrive, () -> m_driverController.getLeftX(), () -> m_driverController.getLeftY(), () -> m_driverController.getRightX(), 1)
    );
    
    //m_TheBassSubsystem.setDefaultCommand(m_TheBassSubsystem.getHoldPositionCommand(() -> m_TheBassSubsystem.getPosition()));


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
    // driverRTrigger.whileTrue(m_IntakeSubsystem.getSetIntakePowerCommand(() -> m_driverController.getRightTriggerAxis()));
    // driverLTrigger.whileTrue(m_IntakeSubsystem.getSetIntakePowerCommand(() -> -1 * m_driverController.getLeftTriggerAxis()));
    // driverA.whileTrue(m_IntakeSubsystem.getIntakeCommand());
    // driverB.whileTrue(m_IntakeSubsystem.getOuttakeCommand());
    // driverX.whileTrue(m_IntakeSubsystem.getSlowIntakeCommand());
    // driverY.whileTrue(m_IntakeSubsystem.getSlowOuttakeCommand());

    // // Shooter
    // shooterA.whileTrue(new SequentialCommandGroup(m_ShooterSubsystem.getPrepareLaunchCommand().withTimeout(2), new LaunchNoteCommand(m_ShooterSubsystem, m_IndexerSubsystem)));

    // // Indexer
    // shooterB.whileTrue(m_IndexerSubsystem.getRunForwardCommand());
    // shooterX.whileTrue(m_IndexerSubsystem.getRunBackwardsCommand());
    
    // shooterY.onTrue(m_PneumaticsSubsystem.getToggleTheBassCommand());

    // shooterLBumper.whileTrue(m_PneumaticsSubsystem.getRaiseTheBassCommand());
    // shooterRBumper.whileTrue(m_PneumaticsSubsystem.getDropTheBassCommand());

    driverRBumper.onTrue(m_PneumaticsSubsystem.getRaiseShoulderCommand());
    driverLBumper.onTrue(m_PneumaticsSubsystem.getDropShoulderCommand());

    //actuatorLeftY.whileTrue(m_JawSubsystem.getRotateNoteCommand(() -> -m_actuatorController.getLeftY()));
    //SUGGESTTION -> create Trigger manualIntake = new Trigger(()->m_actuatorController.getLeftTriggerAxis()>OIConstants.kDriverLTriggerDeadband && m_actuatorController.getRightTriggerAxis()>OIConstants.kDriverLTriggerDeadband)
    //then set the double supplier to be m_actuatorController.getRightTriggerAxis() - m_actuatorController.getLeftTriggerAxis()
    actuatorRTrigger.whileTrue(new ParallelCommandGroup(
      m_IndexerSubsystem.getSetPowerCommand(() -> m_actuatorController.getRightTriggerAxis()),
      m_IntakeSubsystem.getSetIntakePowerCommand(() -> m_actuatorController.getRightTriggerAxis())
      //m_JawSubsystem.getIntakeNoteCommand(.7)
    ));

    actuatorLTrigger.whileTrue(new ParallelCommandGroup(
      m_IndexerSubsystem.getSetPowerCommand(() -> -1*m_actuatorController.getLeftTriggerAxis()),
      m_IntakeSubsystem.getSetIntakePowerCommand(-.75)
      //m_JawSubsystem.getIntakeNoteCommand(-.95)
      ));
    
    //actuatorLBumper.whileTrue(m_TheBassSubsystem.getDropTheBassCommand().withTimeout(.7));
    //actuatorRBumper.whileTrue(m_TheBassSubsystem.getGoToAmpOuttakeCommand().withTimeout(.7));

    actuatorLBumper.whileTrue(m_TheBassSubsystem.getSetPowerCommand(-.3));
    actuatorRBumper.whileTrue(m_TheBassSubsystem.getSetPowerCommand(.3));

   

    // actuatorDpadUp.whileTrue(m_TheBassSubsystem.getGoToRestCommand().withTimeout(1));
    // actuatorDpadDown.whileTrue(m_TheBassSubsystem.getDropTheBassCommand().withTimeout(1));
    // actuatorDpadRight.whileTrue(m_TheBassSubsystem.getGoToTransferCommand().withTimeout(1));
    // actuatorDpadLeft.whileTrue(new SequentialCommandGroup(m_TheBassSubsystem.getGoToTransferCommand(), new WaitCommand(.5), new TransferToIndexerCommand(m_IndexerSubsystem, m_IntakeSubsystem)));


    actuatorDpadUp.whileTrue(m_TheBassSubsystem.daltonGoToRestCommand());
    actuatorDpadRight.whileTrue(m_TheBassSubsystem.daltonGoToTransferCommand());
    actuatorDpadLeft.whileTrue(new SequentialCommandGroup(
      m_TheBassSubsystem.daltonGoToTransferCommand(),
       new WaitCommand(.5),
        new TransferToIndexerCommand(m_IndexerSubsystem, m_IntakeSubsystem)));
    actuatorDpadDown.whileTrue(m_TheBassSubsystem.daltonDropTheBassCommand());
    //actuatorDpadDown.whileTrue(m_TheBassSubsystem.getDropTheBassCommand().withTimeout(.5));


    actuatorX.whileTrue(new ParallelCommandGroup(
      m_ShooterSubsystem.getIntakeSourceCommand(),
      m_IndexerSubsystem.getSourceIntakeCommand()
    ));

    actuatorB.whileTrue(m_ShooterSubsystem.getSetShooterPowerCommand(.7));

    // Climber Control
    //actuatorY.whileTrue(m_ClimberSubsystem.getSetClimberPowerCommand(1));
    //actuatorA.whileTrue(m_ClimberSubsystem.getSetClimberPowerCommand(-.6));


    driverDpad.onTrue(m_robotDrive.getResetHeadingCommand(m_driverController.getPOV()));
    
    //April Tags
    driverB.whileTrue(new SequentialCommandGroup(
      m_PneumaticsSubsystem.getRaiseShoulderCommand(),
      new ParallelDeadlineGroup(
        new ParallelCommandGroup(
          new WaitCommand(2),
          new SequentialCommandGroup(/**new OrthagonalizeCommand(m_robotDrive),**/ new DriveToCloseShotCommand(m_robotDrive).withTimeout(2))
        ), 
        m_ShooterSubsystem.getPrepareLaunchCommand()),
        new LaunchNoteCommand(m_ShooterSubsystem, m_IndexerSubsystem)).handleInterrupt(() -> {m_ShooterSubsystem.stop();}));


    driverStart.whileTrue(new SequentialCommandGroup(
      m_PneumaticsSubsystem.getDropShoulderCommand(),
      new ParallelDeadlineGroup(
        new ParallelCommandGroup(
          new WaitCommand(2),
          new SequentialCommandGroup( new AimAndDriveFarCommand(m_robotDrive).withTimeout(2))
        ), 
        m_ShooterSubsystem.getPrepareLaunchCommand()),
        new LaunchNoteCommand(m_ShooterSubsystem, m_IndexerSubsystem)).handleInterrupt(() -> {m_ShooterSubsystem.stop();}));

    driverY.whileTrue(new SequentialCommandGroup(
      m_PneumaticsSubsystem.getDropShoulderCommand(),
      new ParallelDeadlineGroup(
        new ParallelCommandGroup(
          new WaitCommand(2),
          new SequentialCommandGroup(/**new OrthagonalizeCommand(m_robotDrive) ,**/ new DriveToFarShotCommand(m_robotDrive))
        ), 
        m_ShooterSubsystem.getPrepareLaunchCommand()),
        new LaunchNoteCommand(m_ShooterSubsystem, m_IndexerSubsystem)).handleInterrupt(() -> {m_ShooterSubsystem.stop();}));

    // driverX.whileTrue(new SequentialCommandGroup(
    //   m_PneumaticsSubsystem.getRaiseShoulderCommand(),
    //   new ParallelRaceGroup(
    //     new DriveToSourceCommand(m_robotDrive),
    //     m_ShooterSubsystem.getIntakeSourceCommand()
    //   ),
    //   new LaunchNoteCommand(m_ShooterSubsystem, m_IndexerSubsystem)
    // ).handleInterrupt(() -> {
    //   m_ShooterSubsystem.stop();
    // }));

    driverX.whileTrue(m_robotDrive.getSetXCommand());
    driverA.whileTrue(new PointToAprilTagCommand(m_robotDrive));

    //New Stuff that Cole added
    driverRTrigger.whileTrue(new DriveCommand(m_robotDrive, () -> m_driverController.getLeftX(), () -> m_driverController.getLeftY(), () -> m_driverController.getRightX(), .3));
    driverRTrigger.whileTrue(m_ShooterSubsystem.getPrepareLaunchCommand().handleInterrupt(()->m_ShooterSubsystem.stop()));
    driverRTrigger.and(inShootingRange).onTrue(new LaunchNoteCommand(m_ShooterSubsystem, m_IndexerSubsystem));
    //actuatorB.whileTrue(m_JawSubsystem.getIntakeNoteCommand()) TODO: Uncomment when jaw is on

  } 

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    // TrajectoryConfig config = new TrajectoryConfig(
    //     AutoConstants.kMaxSpeedMetersPerSecond,
    //     AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //     // Add kinematics to ensure max speed is actually obeyed
    //     .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    //     // Start at the origin facing the +X direction
    //     new Pose2d(0, 0, new Rotation2d(0)),
    //     // Pass through these two interior waypoints, making an 's' curve path
    //     List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    //     // End 3 meters straight ahead of where we started, facing forward
    //     new Pose2d(3, 0, new Rotation2d(0)),
    //     config);

    // var thetaController = new ProfiledPIDController(
    //     AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);

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
    return autoChooser.getSelected();
  }



  public static XboxController getDriveControlJoystick() {
    return m_driverController;
  }

  public static XboxController getActuatorController() {
    return m_actuatorController;
  }
}
