// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.pipeline;

public class AutonomousShoot extends CommandBase {
  /** Creates a new AutonomousShoot. */
  Launcher m_launcher;
  Vision m_vision;

  NetworkTableEntry m_validSpeedEntry;
  public AutonomousShoot(Vision vision, Launcher launcher) {
    m_launcher = launcher;
    m_vision = vision;
    m_validSpeedEntry = NetworkTableInstance.getDefault().getTable("autoAim").getEntry("speed valid");
    m_validSpeedEntry.setBoolean(false);
    addRequirements(launcher);
    // Use addRequirements() here to declare subsystem dependencies.
  }
  private double correctedSpeed;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_vision.setMode(pipeline.hubAlternate);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!m_vision.getPipelineSetLazy()) return;
    if(!Constants.DEBUG){
      double setSpeed = 120*m_vision.getDistanceVision() + 964;

      correctedSpeed = Math.min(1900, Math.max(1500, setSpeed));
      m_validSpeedEntry.setBoolean(correctedSpeed == setSpeed);
      if(m_vision.isTargetValid()) m_launcher.SetShooterSpeed(correctedSpeed);
    }
    else{
      m_launcher.SetShooterSpeed(1000);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   
    if(m_launcher.getSetSpeed() >= correctedSpeed){return true;}
    else{return false;}
  }
}
