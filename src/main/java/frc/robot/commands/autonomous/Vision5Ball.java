// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.autonomous.drive.StopDriving;
import frc.robot.commands.autonomous.drive.TrajectoryFollowRelative;
import frc.robot.commands.drive.DrivetrainCalibration;
import frc.robot.subsystems.*;

public class Vision5Ball extends AutonomousRoutine {
  /** Creates a new Vision5Ball. */
  public Vision5Ball(Launcher m_launcher, Drivetrain m_drivetrain, Intake m_intake, CargoStaging m_staging, Vision m_vision) {
    
    addCommands(
    
    new DrivetrainCalibration(m_drivetrain),
    new InstantCommand(() -> m_drivetrain.setGyroAngleAdjustment(30)),
    deadline(
      sequence(new AutoExtendIntake(m_intake),
      new SetIntakeRunning(m_intake, 1),
      new TrajectoryFollowRelative(ThreeOrFourBallAutonomous.cargoTwoTrajectory, m_drivetrain),
      //new RunIntakeTimed(m_intake, 1,0.75),
      //new RunLauncherTimed(m_launcher, 1000, 0.5),
      //new AutonomousShoot(m_vision, m_launcher),
      new SetStageingRunning(m_staging, 1),
      new StopDriving(m_drivetrain),
      new WaitCommand(0.75),
      // new RunLauncherTimed(m_launcher, 1000, 0.5),
      //new AutonomousShoot(m_vision, m_launcher),
      //new SetStageingRunning(m_staging, 0), h
      new SetIntakeRunning(m_intake, 1),
      new TrajectoryFollowRelative(ThreeOrFourBallAutonomous.cargoThreeTrajectory, m_drivetrain),
      new StopDriving(m_drivetrain),
    
      // new RunLauncherTimed(m_launcher, 1000, 0.25),
      //new AutonomousShoot(m_vision, m_launcher),
      //new SetStageingRunning(m_staging, 1), h
      //new RunLauncherTimed(m_launcher, 1000, 0.25),
      parallel( 
        new TrajectoryFollowRelative(ThreeOrFourBallAutonomous.toHumanPlayer, m_drivetrain),
        new SetIntakeRunning(m_intake, 1),
        new WaitCommand(0.45),
        sequence(
          new WaitCommand(0.3),
          new SetStageingRunning(m_staging, 0),
          new InstantCommand(m_staging::resetAutoStaging,m_staging),
          new InstantCommand(m_staging::resetAutoStaging)
        )
      ),
       //20
      new SetIntakeRunning(m_intake, 0),
      new TrajectoryFollowRelative(ThreeOrFourBallAutonomous.toFinalLaunch, m_drivetrain),
      new StopDriving(m_drivetrain),
      new SetIntakeRunning(m_intake, 1),
      new SetStageingRunning(m_staging, 1),
      new WaitCommand(1)
      )

    , new AutonomousShoot(m_vision,m_launcher).perpetually())

    );
  }

  @Override
  public String getName(){
    return "Vision based 5 ball";
  }
}
