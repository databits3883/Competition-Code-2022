package frc.robot.commands.autonomous.drive;

import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.subsystems.Drivetrain;

public class TrajectoryFollowAbsolute extends TrajectoryFollowBase{
    
    public TrajectoryFollowAbsolute(Trajectory trajectory, Drivetrain drivetrain) {
        super(
          trajectory,drivetrain,drivetrain::getCurrentPoseEstimate
        );
      }

}
