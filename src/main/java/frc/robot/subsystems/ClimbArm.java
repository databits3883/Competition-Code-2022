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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ClimbConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;


public class ClimbArm extends SubsystemBase implements SafetyOverridable {
  final CANSparkMax m_liftLeader;

  final Encoder m_lengthEncoder;

  final CANSparkMax m_angleWinchMotor;

  final RelativeEncoder m_angleEncoder;


  
  final PowerDistribution m_distributionBoard;
  /** Creates a new ClimbArm. */
  public ClimbArm() {
    //m_liftWinch = new PWMSparkMax(LENGTH_WINCH_CHANNEL);
    m_liftLeader = new CANSparkMax(LENGTH_WINCH_LEADER_CHANNEL, MotorType.kBrushed);
    CANSparkMax liftFollower = new CANSparkMax(LENGTH_WINCH_FOLLOWER_CHANNEL,MotorType.kBrushed);
    liftFollower.follow(m_liftLeader);
    m_liftLeader.setInverted(false);
    

    liftFollower.getForwardLimitSwitch(Type.kNormallyOpen).enableLimitSwitch(true);
    m_liftLeader.getForwardLimitSwitch(Type.kNormallyOpen).enableLimitSwitch(true);
    liftFollower.getReverseLimitSwitch(Type.kNormallyOpen).enableLimitSwitch(true);
    m_liftLeader.getReverseLimitSwitch(Type.kNormallyOpen).enableLimitSwitch(true);

    m_liftLeader.setIdleMode(IdleMode.kBrake);
    liftFollower.setIdleMode(IdleMode.kBrake);

    m_lengthEncoder = new Encoder(LENGTH_ENCODER_A, LENGTH_ENCODER_B);

    m_angleWinchMotor = new CANSparkMax(ANGLE_WINCH_CHANNEL, MotorType.kBrushless);
    m_angleWinchMotor.setSoftLimit(SoftLimitDirection.kForward, 120f); // was 124.64
    m_angleWinchMotor.setSoftLimit(SoftLimitDirection.kReverse, -1); // was 0
    m_angleWinchMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_angleWinchMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_angleWinchMotor.getReverseLimitSwitch(Type.kNormallyClosed).enableLimitSwitch(true);
    m_angleEncoder = m_angleWinchMotor.getEncoder();
    m_distributionBoard = new PowerDistribution();

    m_lengthEncoder.reset();
    m_lengthEncoder.setDistancePerPulse(1);;
    m_angleEncoder.setPosition(0);

    Shuffleboard.getTab("tab 5").addNumber("angle winch", this::getAngleWinch);
    Shuffleboard.getTab("tab 5").addNumber("length winch", this::measureArmLength);

    SafetyOverrideRegistry.getInstance().register(this);
  }

  public void setExtensionSpeed(double speed){
    m_liftLeader.set(-speed);
  }
  public void setAngleWinchSpeed(double speed){
    m_angleWinchMotor.set(-speed);
  }

  public double getAngleWinchCurrent(){
    return m_distributionBoard.getCurrent(ANGLE_WINCH_PD_SLOT);
  }

  public double getAngleWinch(){
    return m_angleEncoder.getPosition();
  }


  public double measureArmLength(){
    return m_lengthEncoder.get();
  }

  public void enableSafety(){
    m_angleWinchMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_angleWinchMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
  }
  public void disableSafety(){
    m_angleWinchMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
    m_angleWinchMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
  }
}
