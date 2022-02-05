// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CargoStaging;

public class StageCargo extends CommandBase {


  private final int m_direction;
  private final CargoStaging m_staging;
  private final double m_time;
  private final Timer m_timer = new Timer();

  /** Creates a new StageCargo. */
  public StageCargo(CargoStaging staging, int direction, double time) {

    m_time = time; 
    m_staging = staging;
    m_direction = direction;

    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(m_staging);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_staging.SetMotorSpeed(m_direction);
    m_timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_staging.SetMotorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {



    return m_timer.hasElapsed(m_time);
  }
}
