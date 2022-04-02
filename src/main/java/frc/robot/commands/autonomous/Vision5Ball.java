// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.autonomous.Tracking.TurnToGoal;
import frc.robot.commands.autonomous.drive.StopDriving;
import frc.robot.commands.autonomous.drive.TrajectoryFollowAbsolute;
import frc.robot.commands.autonomous.drive.TrajectoryFollowRelative;
import frc.robot.commands.drive.AcquireTarget;
import frc.robot.commands.drive.DrivetrainCalibration;
import frc.robot.subsystems.*;

public class Vision5Ball extends AutonomousRoutine {
  Trajectory cargoTwoTrajectory = ThreeOrFourBallAutonomous.cargoTwoTrajectory;
  Trajectory cargoThreeTrajectory = ThreeOrFourBallAutonomous.cargoThreeTrajectory.transformBy(
    cargoTwoTrajectory.sample(cargoTwoTrajectory.getTotalTimeSeconds()).poseMeters.minus( 
    ThreeOrFourBallAutonomous.cargoThreeTrajectory.getInitialPose())
      
  );
  Trajectory toHumanPlayer = ThreeOrFourBallAutonomous.toHumanPlayer.transformBy(
    cargoThreeTrajectory.sample(cargoThreeTrajectory.getTotalTimeSeconds()).poseMeters.minus(
    ThreeOrFourBallAutonomous.toHumanPlayer.getInitialPose()
    )
  );
  Trajectory toFinal = ThreeOrFourBallAutonomous.toFinalLaunch.transformBy(
    toHumanPlayer.sample(toHumanPlayer.getTotalTimeSeconds()).poseMeters.minus(
      ThreeOrFourBallAutonomous.toFinalLaunch.getInitialPose()
    )
  );
    

  /** Creates a new Vision5Ball. */
  public Vision5Ball(Launcher m_launcher, Drivetrain m_drivetrain, Intake m_intake, CargoStaging m_staging, Vision m_vision) {
    
    addCommands(
    
    new DrivetrainCalibration(m_drivetrain),
    new InstantCommand(() -> m_drivetrain.resetGyro()),
    deadline(
      sequence(
        new AutoExtendIntake(m_intake),
        new SetIntakeRunning(m_intake, 1),
        new TrajectoryFollowAbsolute(cargoTwoTrajectory, m_drivetrain),
        new SetStageingRunning(m_staging, 1),
        new TurnToGoal(m_drivetrain, m_vision).withTimeout(0.75),

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
            new SetStageingRunning(m_staging, 0),
            new InstantCommand(m_staging::resetAutoStaging),
            new SetIntakeRunning(m_intake, 1)
          ),
          new TrajectoryFollowAbsolute(toHumanPlayer, m_drivetrain)
        ),
        
        new TrajectoryFollowAbsolute(toFinal, m_drivetrain),
        new SetStageingRunning(m_staging, 1)

        
        
        
        



      )
    , new AutonomousShoot(m_vision,m_launcher).perpetually())

    );
  }

  @Override
  public String getName(){
    return "Vision based 5 ball";
  }
}
