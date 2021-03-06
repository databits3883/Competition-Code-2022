// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.NotifierCommand;
import frc.robot.subsystems.ClimbArm;

import static frc.robot.Constants.ClimbConstants.*;

public class ClimbPull extends NotifierCommand {
  final ClimbArm m_arm;
  /** Creates a new ClimbPull. */
  public ClimbPull(ClimbArm arm) {
    // Use addRequirements() here to declare subsystem dependencies.

    super(()->{
      if(arm.measureArmLength() > ON_BAR_HEIGHT){
        arm.setExtensionSpeed(ARM_PULL_SPEED);
      }else{
        arm.setExtensionSpeed(0);
      }
    }, AUTOCLIMB_CHECK_PERIOD, arm);
    m_arm=arm;
    addRequirements(m_arm);
  }

  @Override
  public void initialize(){
    super.initialize();
    m_arm.setAngleWinchSpeed(WINCH_DESLACK_SPEED);
  }

  @Override
  public void end(boolean interrupted) {
    m_arm.setExtensionSpeed(0);
    m_arm.setAngleWinchSpeed(0);
    super.end(interrupted);
  }

  @Override
  public boolean isFinished(){
    return m_arm.measureArmLength()<=ON_BAR_HEIGHT;
  }
}
