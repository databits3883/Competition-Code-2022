// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Launcher;

public class RunLauncher extends CommandBase {
  /** Creates a new RunLauncher. */
  private final Launcher m_launcher;
  private double m_rpm;
  public RunLauncher(Launcher launcher, double rpm) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_launcher = launcher;
    m_rpm = rpm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_launcher.SetShooterSpeed(m_rpm);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
