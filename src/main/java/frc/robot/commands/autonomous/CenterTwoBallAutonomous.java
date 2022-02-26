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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.autonomous.drive.TrajectoryFollowRelative;
import frc.robot.commands.drive.DrivetrainCalibration;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CenterTwoBallAutonomous extends SequentialCommandGroup {

  private final Trajectory cargoTwoTrajectory = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0,0, new Rotation2d(0)), 

    List.of(
      new Translation2d(2/1.5 ,-1.7/2)
    ),

    new Pose2d(2,-1.7,Rotation2d.fromDegrees(0.0)),
    DriveConstants.CONFIG);




    private final Trajectory originTrajectory = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0,0, new Rotation2d(0)), 

    List.of(
      new Translation2d(-2/2 ,1.7/2)
    ),

    new Pose2d(-2,1.7,Rotation2d.fromDegrees(0.0)),
    DriveConstants.CONFIG);



  /** Creates a new CenterTwoBallAutonomous. */
  public CenterTwoBallAutonomous(Launcher m_launcher, Drivetrain m_drivetrain, Intake m_intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    new DrivetrainCalibration(m_drivetrain),
    new RunLauncherTimed(m_launcher, 1000, 1),
    new SetIntakeRunning(m_intake, -1),
    new RunLauncherTimed(m_launcher, 1000, 1),
    new RunLauncherTimed(m_launcher, 0, 0.01),
    new TrajectoryFollowRelative(cargoTwoTrajectory, m_drivetrain),
    new RunIntakeTimed(m_intake, 1,0.75),
    new TrajectoryFollowRelative(originTrajectory, m_drivetrain),
    new RunLauncherTimed(m_launcher, 1000, 1),
    new RunIntakeTimed(m_intake, -1, 0.25),
    new RunIntakeTimed(m_intake, 1, 1),
    new RunLauncherTimed(m_launcher, 0, 0.01));
  }
}
