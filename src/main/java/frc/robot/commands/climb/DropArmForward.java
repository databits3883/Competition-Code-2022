// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbArm;

import static frc.robot.Constants.ClimbConstants.*;

public class DropArmForward extends CommandBase {
  /** Creates a new DropArmForward. */
  private final ClimbArm m_climbArm;
  private double startTime;
  private double currentTime;
  public DropArmForward(ClimbArm climbArm) {
    m_climbArm = climbArm;


    addRequirements(climbArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climbArm.setAngleWinchSpeed(ARM_DROP_SPEED);
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentTime = Timer.getFPGATimestamp();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climbArm.setAngleWinchSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(currentTime - startTime > ARM_DROP_TIME){
      return true;
    }
    else{
      return false;

    }
    
  }
}
