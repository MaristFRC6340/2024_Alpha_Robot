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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Commands.AimAndDriveFarCommand;
import frc.robot.Commands.DriveCommand;
import frc.robot.Commands.DriveTargetLockCommand;
import frc.robot.Commands.DriveToCloseShotCommand;
import frc.robot.Commands.DriveToFarShotCommand;
import frc.robot.Commands.DriveToSourceCommand;
import frc.robot.Commands.HighLaunchNoteCommand;
import frc.robot.Commands.IntakeUntilNoteCommand;
import frc.robot.Commands.LEDCommand;
import frc.robot.Commands.LaunchNoteCommand;
import frc.robot.Commands.LowLaunchNoteCommand;
import frc.robot.Commands.OrthagonalizeCommand;
import frc.robot.Commands.PointToAprilTagCommand;
import frc.robot.Commands.TransferToIndexerCommand;
import frc.robot.Commands.WaitUntilReadyCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CelloConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.AmpTicklerSubsystem;
import frc.robot.subsystems.CelloSubsystem;
//import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.JawSubsystem;
import frc.robot.subsystems.LEDSubsystem;
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

  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  private final TheBassSubsystem m_TheBassSubsystem = new TheBassSubsystem();

  private final CelloSubsystem m_CelloSubsystem = new CelloSubsystem();

  private final AmpTicklerSubsystem m_AmpTicklerSubsystem = new AmpTicklerSubsystem();

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
  Trigger driverDpadUp = new Trigger(()->m_driverController.getPOV()==0);
  Trigger driverDpadLeft = new Trigger(() -> m_driverController.getPOV()==270);
  Trigger driverDpadRight = new Trigger(() -> m_driverController.getPOV() == 90);
  Trigger driverDpadDown = new Trigger(() -> m_driverController.getPOV()==180);

  Trigger noteDetected = new Trigger(()->m_IntakeSubsystem.hasNote());

  //Trigger inShootingRange = new Trigger(()->m_robotDrive.inShootingRange(m_PneumaticsSubsystem.getShoulderRaised()));

  //Trigger that is true when the robot is able to shoot successfully
  Trigger inShootingRange= new Trigger(() -> m_PneumaticsSubsystem.inShootingRange());

  Trigger targetLocked = new Trigger(()-> SmartDashboard.getBoolean("inSpeakerRange", false));
  //Limelight stuff:

  NetworkTable limTable;
  NetworkTableEntry ledMode;

  SendableChooser<Command> autoChooser;
  LEDSubsystem leds = new LEDSubsystem(161);
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
    NamedCommands.registerCommand("RaiseTheBass", m_TheBassSubsystem.daltonGoToRestCommand().withTimeout(.5));
    NamedCommands.registerCommand("TransferTheBass", m_TheBassSubsystem.daltonGoToTransferCommand().withTimeout(.5));
    NamedCommands.registerCommand("DropTheBass", m_TheBassSubsystem.daltonDropTheBassCommand().withTimeout(1));
    NamedCommands.registerCommand("RaiseShoulder", m_PneumaticsSubsystem.getRaiseShoulderCommand().withTimeout(.5));
    NamedCommands.registerCommand("DropShoulder", m_PneumaticsSubsystem.getDropShoulderCommand().withTimeout(.5));

    NamedCommands.registerCommand("StartIntake", m_IntakeSubsystem.getStartIntakeCommand());
    NamedCommands.registerCommand("StartIntakeSlow", m_IntakeSubsystem.getStartSlowIntakeCommand());
    NamedCommands.registerCommand("StopIntake", m_IntakeSubsystem.getStopIntakeCommand());

    // NamedCommands.registerCommand("IntakeAndShootLow", new IntakeAndShootLowCommand(m_IndexerSubsystem, m_ShooterSubsystem, m_PneumaticsSubsystem));
    // NamedCommands.registerCommand("IntakeAndShootHigh", new IntakeAndShootHighCommand(m_IndexerSubsystem, m_ShooterSubsystem, m_PneumaticsSubsystem));

    NamedCommands.registerCommand("TransferNote", new SequentialCommandGroup(m_TheBassSubsystem.getGoToTransferCommand(), new TransferToIndexerCommand(m_IndexerSubsystem, m_IntakeSubsystem).withTimeout(1.5), m_TheBassSubsystem.getDropTheBassCommand()));


    NamedCommands.registerCommand("PickupAndTransfer", new SequentialCommandGroup(
      new IntakeUntilNoteCommand(m_IntakeSubsystem, false),
      m_TheBassSubsystem.daltonGoToTransferCommand(),
      new WaitCommand(.5),
      new TransferToIndexerCommand(m_IndexerSubsystem, m_IntakeSubsystem)
    ));


    NamedCommands.registerCommand("LaunchNoteKeepShooter", m_IndexerSubsystem.getRunForwardCommand().withTimeout(.4));
    autoChooser = AutoBuilder.buildAutoChooser("default");
    SmartDashboard.putData("Auto Chooser", autoChooser);
     

    // Configure the button bindings
    configureButtonBindings();

    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
        if(alliance.get() == DriverStation.Alliance.Red){
          leds.setDefaultCommand(new LEDCommand(LEDConstants.red(), leds));
        }
        else{
          leds.setDefaultCommand(new LEDCommand(LEDConstants.blue(), leds));

        }
    }
    else{
      leds.setDefaultCommand(new LEDCommand(LEDConstants.ambientManatee(), leds));
    }
    //Set Default Commands
    m_robotDrive.setDefaultCommand(
      new DriveCommand(m_robotDrive, () -> m_driverController.getLeftX(), () -> m_driverController.getLeftY(), () -> m_driverController.getRightX(), 1)
    );




    //Network Table stuff for notifying drivers
    limTable = NetworkTableInstance.getDefault().getTable("limelight");
    ledMode = limTable.getEntry("ledMode");
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
    
    //Old actuator Controls
        //Manual intake control
    // actuatorRTrigger.whileTrue(new ParallelCommandGroup(
    //   m_IndexerSubsystem.getSetPowerCommand(() -> m_actuatorController.getRightTriggerAxis()),
    //   m_IntakeSubsystem.getSetIntakePowerCommand(() -> m_actuatorController.getRightTriggerAxis())
    // ));

    // actuatorLTrigger.whileTrue(new ParallelCommandGroup(
    //   m_IndexerSubsystem.getSetPowerCommand(() -> -1*m_actuatorController.getLeftTriggerAxis()),
    //   m_IntakeSubsystem.getSetIntakePowerCommand(-.75)
    //   ));

    //   //Manually moving the bass up and down
    // actuatorLBumper.whileTrue(m_TheBassSubsystem.getSetPowerCommand(-.3));
    // actuatorRBumper.whileTrue(m_TheBassSubsystem.getSetPowerCommand(.3));



    // //Presets for moving the bass up and down
    // actuatorDpadUp.whileTrue(m_TheBassSubsystem.daltonGoToRestCommand());
    // actuatorDpadRight.whileTrue(m_TheBassSubsystem.daltonGoToTransferCommand());
    // actuatorDpadDown.whileTrue(m_TheBassSubsystem.daltonDropTheBassCommand());

    // //Moves the bass to the transfer position and runs the intake
    // actuatorDpadLeft.whileTrue(new SequentialCommandGroup(
    //   m_TheBassSubsystem.daltonGoToTransferCommand(),
    //    new WaitCommand(.5),
    //     new TransferToIndexerCommand(m_IndexerSubsystem, m_IntakeSubsystem)));


    // //Intake from the source through the shooter
    // actuatorX.whileTrue(new ParallelCommandGroup(
    //   m_ShooterSubsystem.getIntakeSourceCommand(),
    //   m_IndexerSubsystem.getSourceIntakeCommand()
    // ));

    // //Manually spin up shooter
    // actuatorB.whileTrue(m_ShooterSubsystem.getSetShooterPowerCommand(.7));

    // //Raise and lower the shooter tilt
     driverRBumper.onTrue(m_PneumaticsSubsystem.getRaiseShoulderCommand());
     driverLBumper.onTrue(m_PneumaticsSubsystem.getDropShoulderCommand());
    


    //Actuator Controls

    //Moves the note from the shooter to the intake
    actuatorLTrigger.whileTrue(new ParallelCommandGroup(
      m_ShooterSubsystem.getIntakeSourceCommand(),
      m_IndexerSubsystem.getIntakeFromSourceCommand(),
      m_IntakeSubsystem.getOuttakeCommand(),
      m_AmpTicklerSubsystem.getSetSpeedCommand(CelloConstants.kSlowIntakeSpeed)
    ));

    //Moves the note from the intake to the shooter
    actuatorRTrigger.whileTrue(new ParallelCommandGroup(
      new IntakeUntilNoteCommand(m_IntakeSubsystem, m_IntakeSubsystem.hasNote()),
      m_IndexerSubsystem.getRunForwardCommand(),
      m_AmpTicklerSubsystem.getSetSpeedCommand(CelloConstants.kOuttakeSpeed)
    ));

    //Sets amp to the outtake position then runs the outtake when held.
    actuatorY.whileTrue(new SequentialCommandGroup(
      m_CelloSubsystem.getSetPositionCommand(CelloConstants.kOuttakePosition),
      new WaitCommand(.25),
      m_AmpTicklerSubsystem.getSetSpeedCommand(CelloConstants.kOuttakeSpeed)
    ));

    //Sets the amp to the transfer position and the intake to the amp transfer position, then transfers.
    actuatorB.whileTrue(new SequentialCommandGroup(
      m_TheBassSubsystem.getGoToAmpTransferPositonCommand(),
      m_CelloSubsystem.getSetPositionCommand(CelloConstants.kIntakeTransferPosition),
      new WaitCommand(2),
      new ParallelCommandGroup(
        m_AmpTicklerSubsystem.getSetSpeedCommand(CelloConstants.kSlowIntakeSpeed),
        m_IntakeSubsystem.getSlowOuttakeCommand()
      )));

    //Sets the amp to short mode for field traversal
    actuatorA.onTrue(m_CelloSubsystem.getSetPositionCommand(CelloConstants.kTravelPosition));



    //Sets the bass to travelling/up/rest position
    actuatorDpadUp.onTrue(m_TheBassSubsystem.daltonGoToRestCommand());

    //Sets the bass to transfer position and transfers to the indexer
    actuatorDpadLeft.whileTrue(
      new SequentialCommandGroup(
        m_TheBassSubsystem.daltonGoToTransferCommand(),
        new WaitCommand(.25),
        new TransferToIndexerCommand(m_IndexerSubsystem, m_IntakeSubsystem)
      )
    );

    //Sets the bass to intaking height
    actuatorDpadDown.onTrue(
      m_TheBassSubsystem.daltonDropTheBassCommand()
    );

    //Manual control for the bass and cello
    actuatorLeftY.whileTrue(m_TheBassSubsystem.getSetPowerCommand(() -> m_actuatorController.getLeftY()));
    actuatorRightY.whileTrue(m_CelloSubsystem.getSetPowerCommand(() -> {return m_actuatorController.getRightY()*.2;}));

    //Toggles the shooter spinning up
    actuatorRBumper.toggleOnTrue(m_ShooterSubsystem.getSetShooterPowerCommand(.7));

    //Moves the ampersand towards outtake and spins the intake wheels out to get it unstuck
    actuatorLBumper.whileTrue(new ParallelCommandGroup(
      m_CelloSubsystem.getSetPowerCommand(.2),
      m_IntakeSubsystem.getSlowOuttakeCommand()
    ));

    //End of Actuator Controls





    //Reset gyro
    driverDpadUp.onTrue(m_robotDrive.getResetHeadingCommand(m_driverController.getPOV()));
    
    //April Tags

    //Close shot w/ sliding (probably gone soon)
    driverB.whileTrue(new SequentialCommandGroup(
      m_PneumaticsSubsystem.getRaiseShoulderCommand(),
      new ParallelDeadlineGroup(
        new ParallelCommandGroup(
          new WaitCommand(2),
          new SequentialCommandGroup( new DriveToCloseShotCommand(m_robotDrive).withTimeout(2))
        ), 
        m_ShooterSubsystem.getPrepareLaunchCommand()),
        new LaunchNoteCommand(m_ShooterSubsystem, m_IndexerSubsystem)).handleInterrupt(() -> {m_ShooterSubsystem.stop();}));


    //Angles towards april tag and slides forward/backwards to far shot. Beautiful and desireable command.
    driverStart.whileTrue(new SequentialCommandGroup(
      m_PneumaticsSubsystem.getDropShoulderCommand(),
      new ParallelDeadlineGroup(
        new ParallelCommandGroup(
          new WaitCommand(2),
          new SequentialCommandGroup( new AimAndDriveFarCommand(m_robotDrive).withTimeout(2))
        ), 
        m_ShooterSubsystem.getPrepareLaunchCommand()),
        new LaunchNoteCommand(m_ShooterSubsystem, m_IndexerSubsystem)).handleInterrupt(() -> {m_ShooterSubsystem.stop();}));

    //Slides to far shot position. Undesireable command, probably gone soon.
    driverY.whileTrue(new SequentialCommandGroup(
      m_PneumaticsSubsystem.getDropShoulderCommand(),
      new ParallelDeadlineGroup(
        new ParallelCommandGroup(
          new WaitCommand(2),
          new SequentialCommandGroup( new DriveToFarShotCommand(m_robotDrive))
        ), 
        m_ShooterSubsystem.getPrepareLaunchCommand()),
        new LaunchNoteCommand(m_ShooterSubsystem, m_IndexerSubsystem)).handleInterrupt(() -> {m_ShooterSubsystem.stop();}));

    driverX.whileTrue(m_robotDrive.getSetXCommand());
    driverA.whileTrue(new PointToAprilTagCommand(m_robotDrive));

    //New Stuff that Cole added
    // driverRTrigger.whileTrue(new DriveTargetLockCommand(m_robotDrive, () -> m_driverController.getLeftX(), () -> m_driverController.getLeftY(), () -> m_driverController.getRightX(), .3));
    // driverRTrigger.whileTrue(m_ShooterSubsystem.getPrepareLaunchCommand().handleInterrupt(()->m_ShooterSubsystem.stop()));
    // driverRTrigger.and(inShootingRange).onTrue(new LaunchNoteCommand(m_ShooterSubsystem, m_IndexerSubsystem));


    driverDpadDown.toggleOnTrue(new DriveTargetLockCommand(
      m_robotDrive,
     () -> m_driverController.getLeftX(),
    () -> m_driverController.getLeftY(),
    () -> m_driverController.getRightX()));
    

    inShootingRange.onTrue(
      new ParallelCommandGroup(new InstantCommand(() -> ledMode.setDouble(3.0)), new LEDCommand(LEDConstants.flashingGreen(), leds)
)
    );
    inShootingRange.onFalse(
      new InstantCommand(() -> ledMode.setDouble(0))
    );

    driverLTrigger.whileTrue(new SequentialCommandGroup(
        m_ShooterSubsystem.getPrepareLaunchCommand().withTimeout(.1),
        new WaitUntilReadyCommand(() -> m_PneumaticsSubsystem.getShoulderRaised()),
        new LaunchNoteCommand(m_ShooterSubsystem, m_IndexerSubsystem)
    ).handleInterrupt(() -> {m_ShooterSubsystem.stop();}));

    targetLocked.whileTrue(new LEDCommand(LEDConstants.green(), leds));
    noteDetected.whileTrue(new LEDCommand(LEDConstants.orange(), leds));

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
