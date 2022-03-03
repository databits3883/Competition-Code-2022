// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.NotifierCommand;
import frc.robot.subsystems.Intake;

import static frc.robot.Constants.IntakeConstants.*;

public class SetIntakeToMid extends NotifierCommand {
  /** Creates a new SetIntakeToMid. */
  final Intake m_arm;
  /** Creates a new ClimbPull. */
  public SetIntakeToMid(Intake arm) {
    // Use addRequirements() here to declare subsystem dependencies.

    super(()->{
      if(arm.getDrawHeight() > MID_LEVEL){
        arm.runDrawAtSpeed(0.5);
      }else{
        arm.runDrawAtSpeed(0);
      }
    }, 5, arm);
    m_arm=arm;
    addRequirements(m_arm);
  }

  @Override
  public void end(boolean interrupted) {
    m_arm.runDrawAtSpeed(0);
    super.end(interrupted);
  }

  @Override
  public boolean isFinished(){
    return m_arm.getDrawHeight()<=MID_LEVEL;
  }
}

