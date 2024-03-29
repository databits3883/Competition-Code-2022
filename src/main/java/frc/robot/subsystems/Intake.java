// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AsynchronousInterrupt;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.IntakeConstants.*;

import com.fasterxml.jackson.databind.type.ResolvedRecursiveType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Intake extends SubsystemBase implements SafetyOverridable {
  public static boolean IntakeDisableFlag = false;

  private final CANSparkMax m_takeMotor;
  
  private final CANSparkMax m_raiseMotor;
  private final RelativeEncoder m_raiseEncoder;

  private final SparkMaxPIDController m_raiseController;

  static final int RAISE_SLOT = 0;
  static final int LOWER_SLOT =1;

  private double lastInSpeed =0;
  private double lastLiftSpeed =0;


  public Intake() {
    m_takeMotor  = new CANSparkMax(INTAKE_CHANNEL,MotorType.kBrushless);
    m_takeMotor.setIdleMode(IdleMode.kBrake);
    m_takeMotor.setInverted(true);


    m_raiseMotor = new CANSparkMax(RAISE_LOWER_CHANNEL, MotorType.kBrushless);
    m_raiseMotor.setSoftLimit(SoftLimitDirection.kForward, 0.0f);
    m_raiseMotor.setSoftLimit(SoftLimitDirection.kReverse, REVERSE_SOFT_LIMIT);
    m_raiseMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_raiseMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_raiseMotor.setInverted(true);
    m_raiseEncoder = m_raiseMotor.getEncoder();
    m_raiseController = m_raiseMotor.getPIDController();

    m_raiseMotor.setIdleMode(IdleMode.kBrake);
    m_raiseEncoder.setPosition(0);

    m_raiseController.setSmartMotionMaxAccel(MAX_RAISE_ACCEL, RAISE_SLOT);
    m_raiseController.setSmartMotionMaxAccel(MAX_LOWER_ACCEL, LOWER_SLOT);
    m_raiseController.setSmartMotionMaxVelocity(MAX_RAISE_VEL, RAISE_SLOT);
    m_raiseController.setSmartMotionMaxAccel(MAX_LOWER_VEL, LOWER_SLOT);


    m_raiseController.setP(0,RAISE_SLOT);
    m_raiseController.setI(0,RAISE_SLOT);
    m_raiseController.setD(0,RAISE_SLOT);
    m_raiseController.setFF(0,RAISE_SLOT);

    m_raiseController.setP(0,LOWER_SLOT);
    m_raiseController.setI(0,LOWER_SLOT);
    m_raiseController.setD(0,LOWER_SLOT);
    m_raiseController.setFF(0,LOWER_SLOT);
    
    Shuffleboard.getTab("tab5").addNumber("Intake angle", m_raiseEncoder::getPosition);

    SafetyOverrideRegistry.getInstance().register(this);
  }
  
  public void takeInOrOut(double speed){
    if(speed!=0) IntakeDisableFlag = false;
    if(speed != lastInSpeed){ 
      m_takeMotor.set(speed);
      lastInSpeed = speed;
    }
  }

  public double getEncoder(){
   return m_raiseEncoder.getPosition();
  }

  public void setIntakeLowerSpeed(double speed){
    if(speed != lastLiftSpeed){
      m_raiseMotor.set(speed);
      lastLiftSpeed = speed;
    }
  }

  private REVLibError setHeight(double height){
    if(height >=m_raiseEncoder.getPosition()){
      return m_raiseController.setReference(height, ControlType.kPosition, RAISE_SLOT);
    }else{
      return m_raiseController.setReference(height, ControlType.kPosition,LOWER_SLOT);
    }
  }

  private void setHeightNew(double length){
    
    if (m_raiseEncoder.getPosition() < length){
      m_raiseMotor.set(.2);


    }
    else{
      m_raiseMotor.set(0);
    }
  }

  public void EnableSoftLimits(boolean enabled){
    m_raiseMotor.enableSoftLimit(SoftLimitDirection.kForward, enabled);
    m_raiseMotor.enableSoftLimit(SoftLimitDirection.kReverse, enabled);
  }

  /*public REVLibError extend(){
    return setHeight(EXTEND_LEVEL);
  }
  public REVLibError retract(){
    return setHeight(RETRACT_LEVEL);
  }
  */
  public void extend(){
    setHeightNew(INTAKE_EXTENDED_DISTANCE);
  }
  public void retract(){
    setHeightNew(INTAKE_RETRACTED_DISTANCE);
  }

  public REVLibError setToMid(){
    return setHeight(MID_LEVEL);
  }

  public boolean atTarget(){
    return Math.abs(m_raiseMotor.getAppliedOutput()) <=MAX_LEVEL_STABLE_OUTPUT;
  }

  public void runDrawAtSpeed(double speed){
    m_raiseMotor.set(speed);
  }
  public double getDrawHeight(){
    return m_raiseEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(IntakeDisableFlag){
      m_takeMotor.set(0);
    }

  }

  public void enableSafety(){
    m_raiseMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_raiseMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
  }
  public void disableSafety(){
    m_raiseMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
    m_raiseMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
  }


}
