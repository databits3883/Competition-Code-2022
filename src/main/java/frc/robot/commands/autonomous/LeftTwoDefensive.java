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
public class LeftTwoDefensive extends AutonomousRoutine {

  


  private final Trajectory cargoTwoTrajectory = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0,0, new Rotation2d(0)), 

    List.of(
      new Translation2d(0.0 , 0.65)
    ),

    new Pose2d(1.3,0.65,Rotation2d.fromDegrees(0.0)),
    DriveConstants.CONFIG);

    private final Trajectory redBallOneTrajectory = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0,0, new Rotation2d(0)), 

    List.of(
      new Translation2d(-0.01/2 ,-1.3/2)
    ),

    new Pose2d(-0.01,-1.3,Rotation2d.fromDegrees(-40)),
    DriveConstants.CONFIG);

    private final Trajectory redBallOneDropOffTrajectory = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0,0, new Rotation2d(0)), 

    List.of(
      new Translation2d(-1.25/2 ,0.5/2)
    ),

    new Pose2d(-1.25,0.5,Rotation2d.fromDegrees(65)),
    DriveConstants.CONFIG);

    private final Trajectory toCalibrateTrajectory = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0,0, new Rotation2d(0)), 

    List.of(
      new Translation2d(0.01/2 ,0.01/2)
    ),

    new Pose2d(-0.01,0.01,Rotation2d.fromDegrees(-150)),
    DriveConstants.CONFIG);

  /** Creates a new BasicAutonomous. */
  public LeftTwoDefensive(Launcher m_launcher, Drivetrain m_drivetrain, Intake m_intake, CargoStaging m_staging) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DrivetrainCalibration(m_drivetrain),
      new InstantCommand(() -> m_drivetrain.setGyroAngleAdjustment(30)),
      //new RunLauncherTimed(m_launcher, 1725, 1),
      //new SetStageingRunning(m_staging, 1),
      //new RunLauncherTimed(m_launcher, 1725, 1),
      new AutoExtendIntake(m_intake),
      new SetStageingRunning(m_staging, 0),
      new SetIntakeRunning(m_intake, 1),
      new TrajectoryFollowRelative(cargoTwoTrajectory, m_drivetrain),
      //new RunIntakeTimed(m_intake, 1,0.75),
      new WaitCommand(0.5),
      //new TrajectoryFollowRelative(originTrajectory, m_drivetrain),
      new StopDriving(m_drivetrain),
      new RunLauncherTimed(m_launcher, /*1825*/0, 0.5),
      new SetStageingRunning(m_staging, 1),
      new RunLauncherTimed(m_launcher, /*1825*/0, 3),
      new RunLauncherTimed(m_launcher, 0, 0.01),
      new TrajectoryFollowRelative(redBallOneTrajectory, m_drivetrain),
      new SetStageingRunning(m_staging, -1),
      new WaitCommand(0.25),
      new SetIntakeRunning(m_intake, 0),
      new TrajectoryFollowRelative(redBallOneDropOffTrajectory, m_drivetrain),
      new SetIntakeRunning(m_intake, -1),
      new TrajectoryFollowRelative(toCalibrateTrajectory, m_drivetrain)
    );
  }

  @Override
  public String getName(){

    return "Left Two Defensive";

  }
}
