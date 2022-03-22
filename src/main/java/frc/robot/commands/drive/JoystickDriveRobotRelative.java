// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.Drivetrain;

public class JoystickDriveRobotRelative extends JoystickDrive {
  /** Creates a new JoystickDriveRobotRelative. */
  public JoystickDriveRobotRelative(Drivetrain drivetrain, Joystick joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(drivetrain,joystick);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.setChassisSpeed(StickFilter.getCurrentCommand());
  }

}
