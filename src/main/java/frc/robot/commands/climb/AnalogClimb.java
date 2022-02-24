// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbArm;

import static frc.robot.Constants.ClimbConstants.*;
public class AnalogClimb extends CommandBase {
  ClimbArm m_arm;

  DoubleSupplier m_pullSpeed;
  DoubleSupplier m_tiltSpeed;

  /** Creates a new AnalogClimb. */
  public AnalogClimb(ClimbArm arm, DoubleSupplier pullSpeed, DoubleSupplier tiltSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = arm;
    m_pullSpeed = pullSpeed;
    m_tiltSpeed = tiltSpeed;

    addRequirements(arm);
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.setAngleWinchSpeed(m_tiltSpeed.getAsDouble() * MANUAL_TILT_SPEED);
    m_arm.setExtensionSpeed(m_tiltSpeed.getAsDouble() * MANUAL_PULL_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
