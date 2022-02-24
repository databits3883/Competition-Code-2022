// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class JoystickDrive extends CommandBase {
  protected final Drivetrain m_drivetrain;

  private final DoubleSupplier m_forwardAxis;
  private final DoubleSupplier m_sideAxis;
  private final DoubleSupplier m_rotateAxis;

  ChassisSpeeds m_target = new ChassisSpeeds();

  /** Creates a new JoystickDrive. */
  public JoystickDrive(Drivetrain drivetrain, Joystick stick) {
    m_drivetrain = drivetrain;
    m_forwardAxis = ()->-stick.getY();
    m_sideAxis = ()->-stick.getX();
    m_rotateAxis =()-> -stick.getTwist();
    addRequirements(drivetrain);
  }


  void getCommandedSpeed(){
    double vx = m_forwardAxis.getAsDouble();
    double vy = m_sideAxis.getAsDouble();

    double squareMag = vx*vx+vy*vy;
    if(squareMag > 1){
      double mag = Math.sqrt(squareMag);
      vx/=mag;
      vy/=mag;
    }
    vy*=Constants.DriveConstants.MAX_WHEEL_SPEED;
    vx*=Constants.DriveConstants.MAX_WHEEL_SPEED;

    double omega = m_rotateAxis.getAsDouble();
    omega*=Constants.DriveConstants.MAX_TURN_SPEED;

    m_target.vxMetersPerSecond = vx;
    m_target.vyMetersPerSecond = vy;
    m_target.omegaRadiansPerSecond = omega;
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    getCommandedSpeed();
    m_drivetrain.setSpeedFieldRelative(m_target);;
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


