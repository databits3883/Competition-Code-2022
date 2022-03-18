// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;

public class StopDriving extends InstantCommand {

  public StopDriving(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(()->drivetrain.setChassisSpeed(new ChassisSpeeds()), drivetrain);
  }
}
