// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ClimbConstants.*;


public class ClimbArm extends SubsystemBase {
  final MotorController m_armLengthWinchMotor;
  final MotorController m_armAngleWinchMotor;
  final Encoder m_armLengthEncoder;
  final Encoder m_armAngleEncoder;

  final DigitalInput m_springHookDetector;

  

  /** Creates a new ClimbArm. */
  public ClimbArm() {}


  public double measureArmLength(){
    return m_armLengthEncoder.getDistance();
  }

  public double measureArmWinchLength(){
    return m_armAngleEncoder.getDistance();
  }
  public boolean getHookDetector(){
    return m_springHookDetector.get();
  }

  @Override
  public void periodic() {
    
  }
}
