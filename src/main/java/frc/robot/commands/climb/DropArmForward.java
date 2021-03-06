// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.NotifierCommand;
import frc.robot.subsystems.ClimbArm;

import static frc.robot.Constants.ClimbConstants.*;

public class DropArmForward extends NotifierCommand {
  /** Creates a new DropArmForward. */
  private final ClimbArm m_climbArm;
  public DropArmForward(ClimbArm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(()->{
      if(arm.getAngleWinch() > PAST_NEXT_BAR_ANGLE){
        arm.setAngleWinchSpeed(ARM_DROP_SPEED);
      }else{
        arm.setAngleWinchSpeed(0);
      }
    }, AUTOCLIMB_CHECK_PERIOD, arm);
    m_climbArm=arm;
    addRequirements(m_climbArm);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climbArm.setAngleWinchSpeed(0);
    super.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_climbArm.getAngleWinch() <= PAST_NEXT_BAR_ANGLE;
  }
}
