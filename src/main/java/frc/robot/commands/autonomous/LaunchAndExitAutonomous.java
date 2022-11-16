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
import frc.robot.commands.autonomous.drive.StopDriving;
import frc.robot.commands.autonomous.drive.TrajectoryFollowBase;
import frc.robot.commands.autonomous.drive.TrajectoryFollowRelative;
import frc.robot.commands.drive.DrivetrainCalibration;
import frc.robot.subsystems.CargoStaging;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LaunchAndExitAutonomous extends AutonomousRoutine {



  private final Trajectory exitTrajectory = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0,0, new Rotation2d(0)), 

    List.of(
      new Translation2d(1.0 ,0.05)
    ),

    new Pose2d(2.0,0.1,Rotation2d.fromDegrees(0.0)),
    DriveConstants.CONFIG);


  /** Creates a new BasicAutonomous. */
  public LaunchAndExitAutonomous(Launcher m_launcher, Drivetrain m_drivetrain, Intake m_intake, CargoStaging m_staging) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //new TrajectoryFollowRelative(zeroTrajectory , m_drivetrain),
      new DrivetrainCalibration(m_drivetrain),
      new InstantCommand(() -> m_drivetrain.setGyroAngleAdjustment(0)),
      new RunLauncherTimed(m_launcher, 1550, 1),
      new SetStageingRunning(m_staging, 1),
      new RunLauncherTimed(m_launcher, 1550, 1),
      new AutoExtendIntake(m_intake),
      new SetStageingRunning(m_staging, 0),
      new SetIntakeRunning(m_intake, 1),
      new TrajectoryFollowRelative(exitTrajectory, m_drivetrain)
      //new RunIntakeTimed(m_intake, 1,0.75),

    );
  }

  @Override
  public String getName(){
    return "Launch and Exit";
  }
}
