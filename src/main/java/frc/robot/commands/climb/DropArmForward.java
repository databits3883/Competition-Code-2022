// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbArm;

public class DropArmForward extends CommandBase {
  /** Creates a new DropArmForward. */
  private final ClimbArm m_climbArm;
  private final double m_speed;
  private final double m_waitTime;
  private double startTime;
  private double currentTime;
  public DropArmForward(ClimbArm climbArm, double speed, double waitTime) {
    m_climbArm = climbArm;
    m_speed = speed;
    m_waitTime = waitTime;


    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climbArm.setAngleWinchSpeed(m_speed);
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
    if(currentTime - startTime > m_waitTime){
      return true;
    }
    else{
      return false;

    }
    
  }
}
