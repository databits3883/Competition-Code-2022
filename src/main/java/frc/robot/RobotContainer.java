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
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
//import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
//import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.*;

import frc.robot.commands.autonomous.*;
import frc.robot.commands.autonomous.drive.TrajectoryFollowRelative;
import frc.robot.commands.drive.*;
import frc.robot.commands.climb.*;
import frc.robot.subsystems.CargoStaging;
import frc.robot.subsystems.ClimbArm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.SafetyOverrideRegistry;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // public static DoubleLogEntry speedEntry;
  // public static DoubleLogEntry tyEntry;
  // public static DoubleLogEntry distanceEntry;

  private SendableChooser<Command> m_autonomousChooser = new SendableChooser<>();

  // The robot's subsystems and commands are defined here...
  
  private final Command m_defaultAutonomous = new PrintCommand("No Autonomous Selected");
  private final Joystick m_stick = new Joystick(0);
  private final Joystick m_copilot = new Joystick(1);


  private final Intake m_intake = new Intake();
  private final CargoStaging m_staging = new CargoStaging();
  private final Launcher m_launcher = new Launcher();
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final Vision m_vision = new Vision();
  private final ClimbArm m_climb = new ClimbArm();


  private final Command m_leftTwoBallAutonomous = new LeftTwoBallAutonomous(m_launcher, m_drivetrain, m_intake,m_staging);
  private final Command m_middleTwoBallAutonomous = new CenterTwoBallAutonomous(m_launcher, m_drivetrain, m_intake,m_staging);
  private final Command m_rightTwoBallAutonomous = new RightTwoBallAutonomous(m_launcher, m_drivetrain, m_intake, m_staging);
  private final Command m_launchAndExitAutonomous = new LaunchAndExitAutonomous(m_launcher, m_drivetrain, m_intake, m_staging);
  private final Command m_threeOrFourBallAutonomous = new ThreeOrFourBallAutonomous(m_launcher, m_drivetrain, m_intake, m_staging, m_vision);
  private final Command m_visionAuto = new Vision5Ball(m_launcher, m_drivetrain, m_intake, m_staging, m_vision);
  private final Command m_leftVisionTwoball = new LeftTwoBallVision(m_launcher, m_drivetrain, m_intake, m_staging,m_vision);


  private final Command m_extendIntake = new ExtendIntake(m_intake);
  private final Command m_autLowerIntakeCommand = new AutoExtendIntake(m_intake);



  private final JoystickButton m_aimButton = new JoystickButton(m_stick, 1); //Ralph said this was right blame him not McKinney

  private final JoystickButton m_intakeButton = new JoystickButton(m_copilot, 10);
  private final JoystickButton m_outtakeButton = new JoystickButton(m_copilot, 8);
  private final JoystickButton m_stageInButton = new JoystickButton(m_copilot, 9);
  private final JoystickButton m_stageOutButton = new JoystickButton(m_copilot, 8);
  private final JoystickButton m_setShooterMax = new JoystickButton(m_stick,12);

  private final JoystickButton m_calibrationButton = new JoystickButton(m_stick, 8);


  private final JoystickButton m_lowerLaunchToggle = new JoystickButton(m_stick, 3);

  private final JoystickButton m_upperLaunchToggle = new JoystickButton(m_stick, 4);
  //private final JoystickButton m_testLowerIntake = new JoystickButton(m_copilot, 8);

  


  




  

  private final Command m_manualDrive = new JoystickDrive(m_drivetrain, m_stick);
  private final Command m_aimDrive = new AcquireTarget(m_stick, m_drivetrain, m_vision,m_launcher);

  private final Command m_manualClimb = new DigitalClimb(m_climb,
  ()->m_copilot.getRawButton(6),
  ()->m_copilot.getRawButton(7),
  ()->m_copilot.getRawButton(4),
  ()->m_copilot.getRawButton(5));

  private final Command m_calibrateDrivetrain = new DrivetrainCalibration(m_drivetrain);
  //private final Command m_simpleAutonomous = new BasicAutonomous(m_launcher, m_drivetrain, m_intake);

  //Needs a more descriptive name
  private final Trajectory m_trajectory = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0,0, new Rotation2d(0)), 

    List.of(
      new Translation2d(1.5,0)
    ),

    new Pose2d(3,0,Rotation2d.fromDegrees(90)),
    DriveConstants.CONFIG);

  private final TrajectoryFollowRelative m_exitTarmacAutonomous = new TrajectoryFollowRelative(m_trajectory, m_drivetrain);
  private final RunIntake m_takeIn = new RunIntake(m_intake,1);
  private final RunIntake m_takeOut = new RunIntake(m_intake,-1);

  //TODO:remove
  private final StartEndCommand m_manualStageIn = new StartEndCommand(
    m_staging::runIn, ()->{m_staging.stop();m_staging.resetAutoStaging();}, m_staging);
  private final StartEndCommand m_manualStageOut = new StartEndCommand(
      m_staging::runOut, ()->{m_staging.stop();m_staging.resetAutoStaging();}, m_staging);
  //end remove


  private final Command m_upperShoot = new StartEndCommand(()->m_launcher.SetShooterSpeed(1550),()->m_launcher.setDutyCycle(0), m_launcher);
  private final Command m_lowerShoot = new StartEndCommand(()->m_launcher.SetShooterSpeed(1000),()->m_launcher.setDutyCycle(0), m_launcher);


  private final RaiseOverMid m_raiseClimbOverMid = new RaiseOverMid(m_climb);
  private final ClimbPull m_pullOntoBar = new ClimbPull(m_climb);
  private final Command m_prepNextBar = new SequentialCommandGroup(
    new ReleaseOffBar(m_climb),
    new DropArmForward(m_climb),
    new ExtendClimbArm(m_climb)
  );
  private final Command m_tiltOntoBar = new RetractIntoBar(m_climb);

  /*private final Command m_pullClimbToZero = new StartEndCommand(()->{m_climb.setAngleWinchSpeed(0.5);m_climb.setExtensionSpeed(-0.4);},
  ()->{m_climb.setAngleWinchSpeed(0);m_climb.setExtensionSpeed(0);}, m_climb);*/
  private final JoystickButton m_zeroClimbButton = new JoystickButton(m_copilot, 16);
 

  private final JoystickButton m_toggleClimb = new JoystickButton(m_copilot, 5);

  // private final Trigger m_raiseOverMidButton= new JoystickButton(m_copilot, 11).and(
  //   new Trigger(m_manualClimb::isScheduled).negate());;
  // private final Trigger m_pullOntoNextBarButton = new JoystickButton(m_copilot,12).and(
  //   new Trigger(m_manualClimb::isScheduled).negate());;
  // private final Trigger m_nextBarButton = new JoystickButton(m_copilot, 13).and(
  //   new Trigger(m_manualClimb::isScheduled).negate());;
  // private final Trigger m_tiltOntoBarButton = new JoystickButton(m_copilot, 14).and(
  //   new Trigger(m_manualClimb::isScheduled).negate());;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_drivetrain.setDefaultCommand(m_manualDrive);
    m_climb.setDefaultCommand(m_manualClimb);
  }

private final Button m_drawIntakeInButton = new JoystickButton(m_copilot, 12)
.whileHeld(new StartEndCommand(()->m_intake.runDrawAtSpeed(0.3), ()->m_intake.runDrawAtSpeed(0), m_intake));
private final Button m_drawIntakeOutButton = new JoystickButton(m_copilot, 11)
.whileHeld(new StartEndCommand(()->m_intake.runDrawAtSpeed(-0.3), ()->m_intake.runDrawAtSpeed(0), m_intake));
private final SetIntakeToMid m_perpIntakeForClimb = new SetIntakeToMid(m_intake);
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

    m_aimButton.whileActiveOnce(m_aimDrive);

    m_intakeButton.whenHeld(m_takeIn);
    m_outtakeButton.whenHeld(m_takeOut);
    m_stageInButton.whenHeld(m_manualStageIn);
    m_stageOutButton.whenHeld(m_manualStageOut);
    m_setShooterMax.whileActiveOnce(new InstantCommand(() ->m_launcher.SetMaxSpeed()));

    m_calibrationButton.whenPressed(m_calibrateDrivetrain);

    m_upperLaunchToggle.toggleWhenPressed(m_upperShoot);
    m_lowerLaunchToggle.toggleWhenPressed(m_lowerShoot);

    new JoystickButton(m_copilot, 3)
      .whenPressed(new InstantCommand(SafetyOverrideRegistry.getInstance()::disableSafety))
      .whenReleased(new InstantCommand(SafetyOverrideRegistry.getInstance()::enableSafety))  
    ;

    //m_toggleClimb.toggleWhenPressed(m_manualClimb, false);

    
    //m_toggleClimb.toggleWhenPressed(m_manualClimb, false);


    // d

    // NetworkTableEntry tyE = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty");
    
    // new JoystickButton(m_stick, 7).whenPressed(new InstantCommand(()->{
    //   double ty = tyE.getDouble(0);
    //   double speed = m_launcher.getSetSpeed();
    //   double distance = m_vision.getDistanceVision();
    //   speedEntry.append(speed);
    //   distanceEntry.append(distance);
    //   tyEntry.append(ty);
    //   System.out.println("logged shooting data, "+distance);
    // }));
    }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return AutonomousRoutineRegistry.getInstance().getSelected();
  }
}
