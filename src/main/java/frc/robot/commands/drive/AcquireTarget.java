// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.autonomous.Tracking.TurnToGoal;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class AcquireTarget extends TurnToGoal {

  /** Creates a new AcquireTarget. */
  public AcquireTarget(Joystick stick, Drivetrain drivetrain, Vision vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(drivetrain, vision);
  }
  
  @Override
  public ChassisSpeeds getTranslationalSpeed(){
    return JoystickDrive.StickFilter.getCurrentCommand();
  } 
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
