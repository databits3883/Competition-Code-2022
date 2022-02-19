// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.drive;

import edu.wpi.first.math.trajectory.Trajectory;


import frc.robot.subsystems.Drivetrain;

public class TrajectoryFollowRelative extends TrajectoryFollowBase {

  private final Drivetrain m_drivetrain;
  /** Creates a new TrajectoryFollowRelative. */
  public TrajectoryFollowRelative(Trajectory trajectory, Drivetrain drivetrain) {
    super(
      trajectory,drivetrain,drivetrain::getPoseRelative
    );

    m_drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.setPoseRelative();
    super.initialize();
  }

}
