// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.autonomous.drive.StopDriving;
import frc.robot.commands.autonomous.drive.TrajectoryFollowRelative;
import frc.robot.commands.drive.DrivetrainCalibration;
import frc.robot.subsystems.CargoStaging;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreeOrFourBallAutonomous extends AutonomousRoutine {


  static final Trajectory cargoTwoTrajectory = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0,0, new Rotation2d(0)), 

    List.of(
      new Translation2d(1.11/2 , -0.01)
    ),

    new Pose2d(1.11,-0.01,Rotation2d.fromDegrees(0.0)),
    DriveConstants.CONFIG);

    private final Trajectory originTrajectoryFirst = TrajectoryGenerator.generateTrajectory(
    new Pose2d(1.11,0.01, new Rotation2d(0)), 

    List.of(
      new Translation2d(-1.11/2 ,0.01/2)
    ),

    new Pose2d(-1.3,-0.8,Rotation2d.fromDegrees(0.0)),
    DriveConstants.CONFIG);

    static final Trajectory cargoThreeOrFourTrajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0,0, new Rotation2d(0)), 
  
      List.of(
        new Translation2d(-1/3/2 ,-0.8/2)
      ),
  
      new Pose2d(-1.3,-0.8,Rotation2d.fromDegrees(0.0)),
      DriveConstants.CONFIG);

    static final Trajectory cargoThreeTrajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0,0, new Rotation2d(0)), 
  
      List.of(
        new Translation2d(-0.3, 0),
        new Translation2d(-1.0 ,-2.5/2)
      ),
  
      new Pose2d(-1.2,-2.5,Rotation2d.fromDegrees(-60.d)),
      DriveConstants.CONFIG);
    
      static final Trajectory toHumanPlayer = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0,0, new Rotation2d(0)), 
    
        List.of(
          new Translation2d(3.9/2 ,-1.7/2)
        ),
    
        new Pose2d(3.7,-1.52,Rotation2d.fromDegrees(10)),
        DriveConstants.CONFIG);

      static final Trajectory toFinalLaunch = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0,0, new Rotation2d(0)), 
    
        List.of(
          new Translation2d(-0.3, 0),
          new Translation2d(-4.0/2 ,2.25/2)
        ),
    
        new Pose2d(-3.5,2.25,Rotation2d.fromDegrees(-10)),
        DriveConstants.CONFIG);
  
  /** Creates a new ThreeOrFourBallAutonomous. */
  public ThreeOrFourBallAutonomous(Launcher m_launcher, Drivetrain m_drivetrain, Intake m_intake, CargoStaging m_staging, Vision m_vision) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    //new RunLauncherTimed(m_launcher, 1000, 0.5),
    //new SetStageingRunning(m_staging, 1),
    new DrivetrainCalibration(m_drivetrain),
    new InstantCommand(() -> m_drivetrain.setGyroAngleAdjustment(30)),
    new RunLauncherTimed(m_launcher, 1950, 0),
    new AutoExtendIntake(m_intake),
    new SetStageingRunning(m_staging, 0),
    new SetIntakeRunning(m_intake, 1),
    new TrajectoryFollowRelative(cargoTwoTrajectory, m_drivetrain),
    //new RunIntakeTimed(m_intake, 1,0.75),
    //new RunLauncherTimed(m_launcher, 1000, 0.5),
    //new AutonomousShoot(m_vision, m_launcher),
    new SetStageingRunning(m_staging, 1),
    new StopDriving(m_drivetrain),
    new WaitCommand(0.95),
    new RunLauncherTimed(m_launcher, 1880, 0),
    // new RunLauncherTimed(m_launcher, 1000, 0.5),
    //new AutonomousShoot(m_vision, m_launcher),
    //new SetStageingRunning(m_staging, 0), h
    new TrajectoryFollowRelative(cargoThreeTrajectory, m_drivetrain),
    new StopDriving(m_drivetrain),
    new WaitCommand(0.5),
    
  
    // new RunLauncherTimed(m_launcher, 1000, 0.25),
    //new AutonomousShoot(m_vision, m_launcher),
    //new SetStageingRunning(m_staging, 1), h
    //new RunLauncherTimed(m_launcher, 1000, 0.25),
    new TrajectoryFollowRelative(toHumanPlayer, m_drivetrain),
    new RunLauncherTimed(m_launcher, 1880, 0),
    new WaitCommand(0.1), //20
    new SetStageingRunning(m_staging, 0),
    new WaitCommand(0.5),
    new SetIntakeRunning(m_intake, 0),
    new TrajectoryFollowRelative(toFinalLaunch, m_drivetrain),
    new StopDriving(m_drivetrain),
    new SetIntakeRunning(m_intake, 1),
    new SetStageingRunning(m_staging, 1),
    new RunLauncherTimed(m_launcher, 1880, 1.0)
    //new AutonomousShoot(m_vision, m_launcher)
    
    );
  }

  @Override
  public String getName(){
    return "Three/Four Ball Auto";
  }
}