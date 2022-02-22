// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ClimbConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class ClimbArm extends SubsystemBase {
  final MotorController m_liftWinch;

  final Encoder m_lengthEncoder;

  final CANSparkMax m_angleWinchMotor;


  
  final PowerDistribution m_distributionBoard;
  /** Creates a new ClimbArm. */
  public ClimbArm() {
    m_liftWinch = new PWMSparkMax(LENGTH_WINCH_CHANNEL);

    m_lengthEncoder = new Encoder(LENGTH_ENCODER_A, LENGTH_ENCODER_B);

    m_angleWinchMotor = new CANSparkMax(ANGLE_WINCH_CHANNEL, MotorType.kBrushless);

    m_distributionBoard = new PowerDistribution();

  }

  public void setExtensionSpeed(double speed){
    m_liftWinch.set(speed);
  }
  public void setAngleWinchSpeed(double speed){
    m_angleWinchMotor.set(speed);
  }

  public double getAngleWinchCurrent(){
    return m_distributionBoard.getCurrent(ANGLE_WINCH_PD_SLOT);
  }


  public double measureArmLength(){
    return m_lengthEncoder.get();
  }

  @Override
  public void periodic() {
    
  }
}
