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
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ClimbConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class ClimbArm extends SubsystemBase {
  final CANSparkMax m_lengthWinchLeader;
  final CANSparkMax m_lengthWinchFollower;

  final RelativeEncoder m_lengthEncoder;

  final MotorController m_angleWinchMotor;

  final DigitalInput m_springHookDetector;

  
  final PowerDistribution m_distributionBoard;
  /** Creates a new ClimbArm. */
  public ClimbArm() {
    m_lengthWinchLeader = new CANSparkMax(LENGTH_WINCH_CHANNEL,MotorType.kBrushless);
    m_lengthWinchFollower = new CANSparkMax(LENGTH_WINCH_FOLLOWER_CHANNEL, MotorType.kBrushless);

    m_lengthWinchFollower.follow(m_lengthWinchLeader);
    m_lengthEncoder = m_lengthWinchLeader.getEncoder();
    m_lengthEncoder.setPositionConversionFactor(LENGTH_WINCH_GEARING*LENGTH_WINCH_CIRCUMFRENCE);

    m_angleWinchMotor = new Talon(ANGLE_WINCH_CHANNEL);
    m_springHookDetector = new DigitalInput(SPRING_HOOK_SWITCH_CHANNEL);

    m_distributionBoard = new PowerDistribution();

  }

  public void setExtensionSpeed(double speed){
    m_lengthWinchLeader.set(speed);
  }
  public void setAngleWinchSpeed(double speed){
    m_angleWinchMotor.set(speed);
  }

  public double getAngleWinchCurrent(){
    return m_distributionBoard.getCurrent(ANGLE_WINCH_PD_SLOT);
  }


  public double measureArmLength(){
    return m_lengthEncoder.getPosition();
  }

  public boolean getHookDetector(){
    return m_springHookDetector.get();
  }

  @Override
  public void periodic() {
    
  }
}
