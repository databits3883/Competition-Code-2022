// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbArm;

import static frc.robot.Constants.ClimbConstants.*;

public class DigitalClimb extends CommandBase {
  ClimbArm m_arm;

  BooleanSupplier m_pullDown;
  BooleanSupplier m_letUp;

  BooleanSupplier m_tiltForward;
  BooleanSupplier m_tiltBack;

  /** Creates a new DigitalClimb. */
  public DigitalClimb(ClimbArm arm, BooleanSupplier pullDown, BooleanSupplier letUp, BooleanSupplier tiltForward, BooleanSupplier tiltBack) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = arm;

    m_pullDown = pullDown;
    m_letUp = letUp;
    m_tiltForward = tiltForward;
    m_tiltBack = tiltBack;

    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_letUp.getAsBoolean()){
      m_arm.setExtensionSpeed(MANUAL_EXTEND_SPEED);
    }else if(m_pullDown.getAsBoolean()){
      m_arm.setExtensionSpeed(MANUAL_PULL_SPEED);
    }else{
      m_arm.setExtensionSpeed(0);
    }

    if(m_tiltForward.getAsBoolean()){
      m_arm.setAngleWinchSpeed(MANUAL_TILT_SPEED);
    }else if(m_tiltBack.getAsBoolean()){
      m_arm.setAngleWinchSpeed(-MANUAL_TILT_SPEED);
    }else{
      m_arm.setAngleWinchSpeed(0);
    }
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
