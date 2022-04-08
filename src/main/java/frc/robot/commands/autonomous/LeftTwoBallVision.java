// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import java.util.List;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.autonomous.Tracking.TurnToGoal;
import frc.robot.commands.autonomous.drive.StopDriving;
import frc.robot.commands.autonomous.drive.TrajectoryFollowAbsolute;
import frc.robot.commands.autonomous.drive.TrajectoryFollowBase;
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
public class LeftTwoBallVision extends AutonomousRoutine {



  private final Trajectory cargoTwoTrajectoryRelative = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0,0, new Rotation2d(0)), 

    List.of(
      new Translation2d(0.0 , 0.65)
    ),

    new Pose2d(1.3,0.65,Rotation2d.fromDegrees(0.0)),
    DriveConstants.CONFIG);

    private final Trajectory originTrajectoryRelative = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0,0, new Rotation2d(0)), 

    List.of(
      new Translation2d(-1/3/2 ,-0.8/2)
    ),

    new Pose2d(-1.3,-0.8,Rotation2d.fromDegrees(0.0)),
    DriveConstants.CONFIG);

    private final Trajectory originTrajectoryAbsolute = originTrajectoryRelative.transformBy(
        cargoTwoTrajectoryRelative.sample(cargoTwoTrajectoryRelative.getTotalTimeSeconds()).poseMeters.minus(
            originTrajectoryRelative.getInitialPose()
        )
    );

  /** Creates a new BasicAutonomous. */
  public LeftTwoBallVision(Launcher m_launcher, Drivetrain m_drivetrain, Intake m_intake, CargoStaging m_staging, Vision m_vision) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //new TrajectoryFollowRelative(zeroTrajectory , m_drivetrain),
      new DrivetrainCalibration(m_drivetrain),
      new InstantCommand(() -> m_drivetrain.resetGyro()),
      new RunLauncherTimed(m_launcher, 1825, 1),
      parallel(
          new TurnToGoal(m_drivetrain,m_vision),
          new WaitCommand(2)
      ),
      new SetStageingRunning(m_staging, 1),
      new RunLauncherTimed(m_launcher, 1825, 1),
      new AutoExtendIntake(m_intake),
      new SetStageingRunning(m_staging, 0),
      new SetIntakeRunning(m_intake, 1),
      new TrajectoryFollowAbsolute(cargoTwoTrajectoryRelative, m_drivetrain),
      //new RunIntakeTimed(m_intake, 1,0.75),
      new WaitCommand(0.5),
      new TrajectoryFollowAbsolute(originTrajectoryAbsolute, m_drivetrain),
      new StopDriving(m_drivetrain),
      new RunLauncherTimed(m_launcher, 1825, 0.5),
      new SetStageingRunning(m_staging, 1),
      parallel(
          new TurnToGoal(m_drivetrain,m_vision),
          new WaitCommand(2)
      ),
      new RunLauncherTimed(m_launcher, 1825, 3),
      new RunLauncherTimed(m_launcher, 0, 0.01)
           
      
    );
  }

  @Override
  public String getName(){
    return "Left Two with vision";
  }
}
