// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.*;
import frc.robot.commands.autonomous.CenterTwoBallAutonomous;
import frc.robot.commands.autonomous.LeftTwoBallAutonomous;
import frc.robot.commands.autonomous.RightTwoBallAutonomous;
import frc.robot.commands.autonomous.drive.TrajectoryFollowBase;
import frc.robot.commands.autonomous.drive.TrajectoryFollowRelative;
import frc.robot.commands.drive.*;
import frc.robot.subsystems.CargoStaging;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private SendableChooser<Command> m_autonomousChooser = new SendableChooser<>();

  // The robot's subsystems and commands are defined here...
  
  private final Command m_defaultAutonomous = new PrintCommand("No Autonomous Selected");
  private final Joystick m_stick = new Joystick(0);
  private final Joystick m_copilot = new Joystick(1);


  private final Intake m_intake = new Intake();
  private final Launcher m_launcher = new Launcher();
  //private final CargoStaging m_staging = new CargoStaging();
  private final Drivetrain m_drivetrain = new Drivetrain();

  private final JoystickButton m_intakeButton = new JoystickButton(m_copilot, 1);
  private final JoystickButton m_outtakeButton = new JoystickButton(m_copilot, 2);
  private final JoystickButton m_stageInButton = new JoystickButton(m_stick, 4);
  private final JoystickButton m_stageOutButton = new JoystickButton(m_stick, 5);

  private final JoystickButton m_calibrationButton = new JoystickButton(m_stick, 8);

  private final JoystickButton m_basicLaunchToggle = new JoystickButton(m_copilot, 3);

  private final Command m_manualDrive = new JoystickDrive(m_drivetrain, m_stick);

  private final Command m_calibrateDrivetrain = new DrivetrainCalibration(m_drivetrain);

  private final Command m_twoBallLeftAutonomous = new LeftTwoBallAutonomous(m_launcher, m_drivetrain, m_intake);
  private final Command m_twoBallCenterAutonomous = new CenterTwoBallAutonomous(m_launcher, m_drivetrain, m_intake);
  private final Command m_twoBallRightAutonomous = new RightTwoBallAutonomous(m_launcher, m_drivetrain, m_intake);



  //Needs a more descriptive name
  private final Trajectory m_tarmacExitTrajectory = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0,0, new Rotation2d(0)), 

    List.of(
      new Translation2d(1.5,1)
    ),

    new Pose2d(3,0,Rotation2d.fromDegrees(90)),
    DriveConstants.CONFIG);

  private final TrajectoryFollowRelative m_exitTarmac = new TrajectoryFollowRelative(m_tarmacExitTrajectory, m_drivetrain);
  private final RunIntake m_takeIn = new RunIntake(m_intake,1);
  private final RunIntake m_takeOut = new RunIntake(m_intake,-1);
  // private final StageCargo m_stageIn = new StageCargo(m_staging, 1,0.4);
  // private final StageCargo m_stageOut = new StageCargo(m_staging, -1,0.4);

  private final Command m_basicShoot = new StartEndCommand(()->m_launcher.setDutyCycle(0.3),()->m_launcher.setDutyCycle(0), m_launcher);



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    configureAutonomousRoutines();

    m_drivetrain.setDefaultCommand(m_manualDrive);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    m_stick.setYChannel(1);
    m_stick.setXChannel(0);
    m_stick.setTwistChannel(2);
    m_stick.setThrottleChannel(3);

    m_intakeButton.whenHeld(m_takeIn);
    m_outtakeButton.whenHeld(m_takeOut);
    // m_stageInButton.whenHeld(m_stageIn);
    // m_stageOutButton.whenHeld(m_stageOut);

    m_calibrationButton.whenPressed(m_calibrateDrivetrain);
    m_basicLaunchToggle.toggleWhenPressed(m_basicShoot);

    

  }
  /**Configures the autonomous sendable chooser */
  private void configureAutonomousRoutines(){
    m_autonomousChooser.setDefaultOption("NO AUTONOMOUS", m_defaultAutonomous);
    m_autonomousChooser.addOption("Exit Tarmac", m_exitTarmac);
    m_autonomousChooser.addOption("2 Ball Left", m_twoBallLeftAutonomous);
    m_autonomousChooser.addOption("2 Ball Center", m_twoBallCenterAutonomous);
    m_autonomousChooser.addOption("2 Ball Right", m_twoBallRightAutonomous);

    Shuffleboard.getTab("Game Screen").add(m_autonomousChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autonomousChooser.getSelected();
  }
}
