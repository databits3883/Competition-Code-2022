// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import javax.lang.model.util.ElementScanner6;
import javax.naming.directory.DirContext;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.CargoStaging;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetStageingRunning extends InstantCommand {

  private final CargoStaging m_staging;
  private final int m_direction;

  public SetStageingRunning(CargoStaging staging, int direction) {
    m_staging = staging;
    m_direction = direction;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_staging);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(m_direction > 0){
      m_staging.runIn();
    }
    else if (m_direction < 0){
      m_staging.runOut();
    }
    else{
      m_staging.stop();
    }
  }
}
