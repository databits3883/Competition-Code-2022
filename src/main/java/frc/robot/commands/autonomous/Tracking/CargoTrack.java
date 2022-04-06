// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.Tracking;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class CargoTrack extends CommandBase {
  ProfiledPIDController m_controller;
  
  /** Creates a new CargoTrack. */
  Drivetrain m_drivetrain;
  public CargoTrack(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    
    

    

    m_controller = new ProfiledPIDController(0.1, 0, 0, 
    new TrapezoidProfile.Constraints(2*Math.PI, 2*Math.PI/5));
    //Shuffleboard.getTab("Tracking Horizontal Tune").add("controller", m_controller).withWidget("pid controller");
    addRequirements(drivetrain);
  }

  public ChassisSpeeds getTranslationalSpeed(){
    return new ChassisSpeeds();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   //table = NetworkTableInstance.getDefault().getTable("limelight");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds speeds = getTranslationalSpeed();
   // if() speeds.omegaRadiansPerSecond = m_controller.calculate();
    m_drivetrain.setSpeedFieldRelative(speeds);
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
