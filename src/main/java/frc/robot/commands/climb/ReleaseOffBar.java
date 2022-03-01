// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.NotifierCommand;
import frc.robot.subsystems.ClimbArm;

import static frc.robot.Constants.ClimbConstants.*;

public class ReleaseOffBar extends NotifierCommand {
  final ClimbArm m_arm;
  /** Creates a new ClimbPull. */
  public ReleaseOffBar(ClimbArm arm) {
    // Use addRequirements() here to declare subsystem dependencies.

    super(()->{
      if(arm.measureArmLength() < OFF_BAR_HEIGHT){
        arm.setExtensionSpeed(ARM_EXTEND_SPEED/4.0);
      }else{
        arm.setExtensionSpeed(0);
      }
    }, AUTOCLIMB_CHECK_PERIOD, arm);
    m_arm=arm;
    addRequirements(m_arm);
  }

  @Override
  public void end(boolean interrupted) {
    m_arm.setExtensionSpeed(0);
    super.end(interrupted);
  }

  @Override
  public boolean isFinished(){
    return m_arm.measureArmLength()>=OFF_BAR_HEIGHT;
  }
}
