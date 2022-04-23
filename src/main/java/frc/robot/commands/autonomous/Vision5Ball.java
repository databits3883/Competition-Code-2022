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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.autonomous.Tracking.TurnToGoal;
import frc.robot.commands.autonomous.drive.StopDriving;
import frc.robot.commands.autonomous.drive.TrajectoryFollowAbsolute;
import frc.robot.commands.autonomous.drive.TrajectoryFollowRelative;
import frc.robot.commands.drive.AcquireTarget;
import frc.robot.commands.drive.DrivetrainCalibration;
import frc.robot.subsystems.*;

public class Vision5Ball extends AutonomousRoutine {

  static final Trajectory robotCargoTwoTrajectory = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0,0, new Rotation2d(0)), 

    List.of(
      new Translation2d(1.11/2 , -0.01)
    ),

    new Pose2d(1.11,-0.01,Rotation2d.fromDegrees(0.0)),
    DriveConstants.CONFIG);

    static final Trajectory robotCargoThreeOrFourTrajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0,0, new Rotation2d(0)), 
  
      List.of(
        new Translation2d(-1/3/2 ,-0.4/2)
      ),
  
      new Pose2d(-1.3,-0.4,Rotation2d.fromDegrees(0.0)),
      DriveConstants.CONFIG);

    static final Trajectory robotCargoThreeTrajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0,0, new Rotation2d(0)), 
  
      List.of(
        new Translation2d(-0.3, 0),
        new Translation2d(-1.0 ,-2.5/2)
      ),
  
      new Pose2d(-1.2,-2.5,Rotation2d.fromDegrees(-50.d)),
      DriveConstants.CONFIG);
    
      static final Trajectory robotToHumanPlayer = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0,0, Rotation2d.fromDegrees(10)), 
    
        List.of(
          new Translation2d(3.5/2 ,-1.25/2)
        ),
    
        new Pose2d(3.7,-1.25,Rotation2d.fromDegrees(10)),
        DriveConstants.CONFIG);

      static final Trajectory robotToFinalLaunch = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0,0, new Rotation2d(0)), 
    
        List.of(
          new Translation2d(-0.3, 0),
          new Translation2d(-4.0/2 ,2.25/2)
        ),
    
        new Pose2d(-3.3,2.0,Rotation2d.fromDegrees(-10)),
        DriveConstants.CONFIG);
  

  Trajectory cargoTwoTrajectory = robotCargoTwoTrajectory;
  Trajectory cargoThreeTrajectory = robotCargoThreeTrajectory.transformBy(
    cargoTwoTrajectory.sample(cargoTwoTrajectory.getTotalTimeSeconds()).poseMeters.minus( 
    robotCargoThreeTrajectory.getInitialPose())
      
  );
  Trajectory toHumanPlayer = robotToHumanPlayer.transformBy(
    cargoThreeTrajectory.sample(cargoThreeTrajectory.getTotalTimeSeconds()).poseMeters.minus(
    robotToHumanPlayer.getInitialPose()
    )
  );
  Trajectory toFinal = robotToFinalLaunch.transformBy(
    toHumanPlayer.sample(toHumanPlayer.getTotalTimeSeconds()).poseMeters.minus(
      robotToFinalLaunch.getInitialPose()
    )
  );

  
    

  /** Creates a new Vision5Ball. */
  public Vision5Ball(Launcher m_launcher, Drivetrain m_drivetrain, Intake m_intake, CargoStaging m_staging, Vision m_vision) {
    
    addCommands(
    
    new DrivetrainCalibration(m_drivetrain),
    new InstantCommand(() -> m_drivetrain.resetGyro()),
    parallel(
      sequence(
        new AutoExtendIntake(m_intake),
        new SetIntakeRunning(m_intake, 1),
        new TrajectoryFollowAbsolute(cargoTwoTrajectory, m_drivetrain),
        
        new SetIntakeRunning(m_intake, 1),
        new TurnToGoal(m_drivetrain, m_vision).withTimeout(0.75),
        new WaitCommand(0.1),
        new SetStageingRunning(m_staging, 1),

        new TrajectoryFollowAbsolute(cargoThreeTrajectory, m_drivetrain),
        deadline(
          sequence(
            new WaitCommand(0.2),
            new SetStageingRunning(m_staging, 1)
          ),
          
          new TurnToGoal(m_drivetrain, m_vision).withTimeout(0.75)
        ),
        parallel(
          sequence(
            new WaitCommand(0.1),
            new SetStageingRunning(m_staging, 0),
            new InstantCommand(m_staging::resetAutoStaging),
            new SetIntakeRunning(m_intake, 1)
          ),
          new TrajectoryFollowAbsolute(toHumanPlayer, m_drivetrain)
        ),
        
        new TrajectoryFollowAbsolute(toFinal, m_drivetrain),
        new SetStageingRunning(m_staging, 1),
        new TurnToGoal(m_drivetrain, m_vision)


      )
    , new RunLauncherTimed(m_launcher, 2000, 16)
    )
    );
  }

  @Override
  public String getName(){
    return "Vision based 5 ball";
  }
}
