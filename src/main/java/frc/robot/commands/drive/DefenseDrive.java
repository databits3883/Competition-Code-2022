// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.Drivetrain;

import static frc.robot.Constants.DriveConstants.*;

public class DefenseDrive extends JoystickDrive {
  IntSupplier povGet;

  Translation2d[] centers = {
    new Translation2d(0,0),
    new Translation2d(DRIVE_TRACK_LENGTH/2,0),
    new Translation2d(DRIVE_TRACK_LENGTH/2,-DRIVE_TRACK_WIDTH/2),
    new Translation2d(0,-DRIVE_TRACK_WIDTH/2),
    new Translation2d(-DRIVE_TRACK_LENGTH/2,-DRIVE_TRACK_WIDTH/2),
    new Translation2d(-DRIVE_TRACK_LENGTH/2,0),
    new Translation2d(-DRIVE_TRACK_LENGTH/2,DRIVE_TRACK_WIDTH/2),
    new Translation2d(0,DRIVE_TRACK_WIDTH/2),
    new Translation2d(DRIVE_TRACK_LENGTH/2,DRIVE_TRACK_WIDTH/2)
  };

  /** Creates a new DefenseDrive. */
  public DefenseDrive(Drivetrain drivetrain, Joystick stick) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(drivetrain,stick);
    povGet = ()->stick.getPOV();

  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    getCommandedSpeed();
    int pov = povGet.getAsInt();
    Translation2d centerOfRotation = centers[pov<0?0:pov/45 + 1];
    m_drivetrain.setChassisOffCenter(m_target, centerOfRotation);
  }
}
