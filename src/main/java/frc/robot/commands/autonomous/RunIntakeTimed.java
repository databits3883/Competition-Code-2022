// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class RunIntakeTimed extends CommandBase {

  private final Intake m_intake;
  private final int m_direction;
  private Timer m_timer;
  private double m_time;


  /** Creates a new RunIntakeTimed. */
  public RunIntakeTimed(Intake intake, int direction, double time) {
    m_intake = intake;
    m_direction = direction;
    m_time = time;
    m_timer = new Timer();
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.takeInOurOut(Constants.IntakeConstants.spinSpeed * m_direction);
    m_timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.takeInOurOut(Constants.IntakeConstants.spinSpeed * m_direction);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.takeInOurOut(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_time);
  }
}
