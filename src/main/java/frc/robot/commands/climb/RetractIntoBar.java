// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbArm;
import static frc.robot.Constants.ClimbConstants.*;


public class RetractIntoBar extends CommandBase {
  /** Creates a new RetractIntoBar. */
  private final ClimbArm m_climbArm;
  private final double winchSpeed;
  private double current;
  private boolean isUnderTension;
  public RetractIntoBar(ClimbArm climbArm) {
    m_climbArm = climbArm;
    winchSpeed = 1; //change later
    isUnderTension = false;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climbArm.setAngleWinchSpeed(winchSpeed);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    current = m_climbArm.getAngleWinchCurrent();
    if(current > WINCH_CURRENT_THRESHOLD ){
      isUnderTension = true;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climbArm.setAngleWinchSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(isUnderTension){
      return true;
    }
    else{
      return false;
    }
  }
}
