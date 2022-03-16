// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.Tracking;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.pipeline;

public class TurnToGoal extends CommandBase {
  /** Creates a new TurnToGoal. */

  ProfiledPIDController m_controller;
  
  
  
  //NetworkTableEntry kPEntry = table.getEntry("kP");
  //NetworkTableEntry kIEntry = table.getEntry("kI");
  //NetworkTableEntry kDEntry = table.getEntry("kD");
  

  Drivetrain m_drivetrain;
  Vision m_vision;
  public TurnToGoal(Drivetrain drivetrain, Vision vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    m_vision = vision;

    

    

    m_controller = new ProfiledPIDController(0, 0, 0, 
    new TrapezoidProfile.Constraints(2*Math.PI, 2*Math.PI/5));
    Shuffleboard.getTab("Tracking Horizontal Tune").add("controller", m_controller);
    addRequirements(drivetrain, vision);
  }
  public ChassisSpeeds getTranslationalSpeed(){
    return new ChassisSpeeds();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_vision.setMode(pipeline.hubAlternate);
    System.out.println("runningcmd");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds speeds = getTranslationalSpeed();
    if(m_vision.getPipelineSetLazy() &&m_vision.isTargetValid()) speeds.omegaRadiansPerSecond = m_controller.calculate(m_vision.getHorizontalOffset());
    m_drivetrain.setChassisSpeed(speeds);
    System.out.println(m_controller.calculate(m_vision.getHorizontalOffset()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(!interrupted) m_vision.setMode(pipeline.driverCam);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
