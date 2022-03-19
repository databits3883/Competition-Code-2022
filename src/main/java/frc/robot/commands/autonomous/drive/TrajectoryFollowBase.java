// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.Drivetrain;

import static frc.robot.Constants.DriveConstants.*;

import java.util.function.Supplier;

public abstract class TrajectoryFollowBase extends SwerveControllerCommand {
  protected Pose2d m_finalPose;
  protected Trajectory m_trajectory;
  protected final Supplier<Pose2d> m_poseSupplier;

  protected static double squaredTranslationalTolerance = 0.05;

  /** Creates a new TrajectoryFollowBase. */
  public TrajectoryFollowBase(Trajectory trajectory, Drivetrain drivetrain, Supplier<Pose2d> poseSupplier) {
    super(trajectory,
      poseSupplier,
      KINEMATICS, 
      new PIDController(1, 0, 0), 
      new PIDController(1, 0, 0), 
      new ProfiledPIDController(4, 0, 0, 
        new TrapezoidProfile.Constraints(MAX_TURN_SPEED, MAX_TURN_SPEED*10)),
      drivetrain::setStates,
      drivetrain
      );
      drivetrain.setDisplayTrajectory(trajectory);
      m_trajectory = trajectory;
      m_finalPose = m_trajectory.sample(m_trajectory.getTotalTimeSeconds()).poseMeters;
      m_poseSupplier = poseSupplier;
  }

  @Override
  public boolean isFinished(){
    Pose2d current = m_poseSupplier.get();
    double xErr = current.getX() - m_finalPose.getX();
    double yErr = current.getY() - m_finalPose.getY();
    
    return xErr*xErr + yErr*yErr < squaredTranslationalTolerance && super.isFinished();
  }

}
