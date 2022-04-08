// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.autonomous.Tracking.TurnToGoal;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Vision;

public class AcquireTarget extends TurnToGoal {

  Launcher m_launcher;
  Vision m_vision;

  NetworkTableEntry m_validSpeeEntry;
  /** Creates a new AcquireTarget. */
  public AcquireTarget(Joystick stick, Drivetrain drivetrain, Vision vision, Launcher launcher) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(drivetrain, vision);
    m_launcher = launcher;
    m_vision = vision;
    m_validSpeeEntry = NetworkTableInstance.getDefault().getTable("autoAim").getEntry("speed valid");
    m_validSpeeEntry.setBoolean(false);
    addRequirements(launcher);
  }
  @Override
  public void initialize(){
    super.initialize();
    m_vision.resetSnapshot();
  }
  @Override
  public void execute(){
    super.execute();
    //double setSpeed = 120*m_vision.getDistanceVision() + 964;

    //double correctedSpeed = Math.min(1900, Math.max(1500, setSpeed));
    //m_validSpeeEntry.setBoolean(correctedSpeed == setSpeed);
    //if(m_vision.isTargetValid()) m_launcher.SetShooterSpeed(correctedSpeed);
    m_launcher.SetShooterSpeed(1650);
  }
  @Override
  public ChassisSpeeds getTranslationalSpeed(){
    return JoystickDrive.StickFilter.getCurrentCommand();
  } 
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted){
    m_vision.takeSnapshot();
    super.end(interrupted);
  }
}
