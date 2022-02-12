// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.drive;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.Drivetrain;

import static frc.robot.Constants.DriveConstants.*;

public class TrajectoryFollowBase extends SwerveControllerCommand {
  /** Creates a new TrajectoryFollowBase. */
  public TrajectoryFollowBase(Trajectory trajectory, Drivetrain drivetrain) {
    super(trajectory,
      drivetrain::getCurrentPoseEstimate, 
      KINEMATICS, 
      new PIDController(1, 0, 0), 
      new PIDController(1, 0, 0), 
      new ProfiledPIDController(1, 0, 0, 
        new TrapezoidProfile.Constraints(MAX_TURN_SPEED, MAX_TURN_SPEED/100)),
      drivetrain::setStates,
      drivetrain
      );
  }

}
