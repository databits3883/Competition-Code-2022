// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AsynchronousInterrupt;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.IntakeConstants.*;

import com.fasterxml.jackson.databind.type.ResolvedRecursiveType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Intake extends SubsystemBase {
  private final CANSparkMax m_takeMotor;
  
  private final CANSparkMax m_raiseMotor;
  private final RelativeEncoder m_raiseEncoder;

  private final SparkMaxPIDController m_raiseController;

  static final int RAISE_SLOT = 0;
  static final int LOWER_SLOT =1;


  public Intake() {
    m_takeMotor  = new CANSparkMax(INTAKE_CHANNEL,MotorType.kBrushless);
    m_takeMotor.setIdleMode(IdleMode.kBrake);


    m_raiseMotor = new CANSparkMax(RAISE_LOWER_CHANNEL, MotorType.kBrushless);
    m_raiseEncoder = m_raiseMotor.getEncoder();
    m_raiseController = m_raiseMotor.getPIDController();

    m_raiseMotor.setIdleMode(IdleMode.kBrake);

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
    

  }
  
  public void takeInOurOut(double speed){
    m_takeMotor.set(speed);
  }

  private REVLibError setHeight(double height){
    if(height >=m_raiseEncoder.getPosition()){
      return m_raiseController.setReference(height, ControlType.kPosition, RAISE_SLOT);
    }else{
      return m_raiseController.setReference(height, ControlType.kPosition,LOWER_SLOT);
    }
  }

  public REVLibError extend(){
    return setHeight(EXTEND_LEVEL);
  }
  public REVLibError retract(){
    return setHeight(RETRACT_LEVEL);
  }
  public REVLibError setToMid(){
    return setHeight(MID_LEVEL);
  }

  public boolean atTarget(){
    return Math.abs(m_raiseMotor.getAppliedOutput()) <=MAX_LEVEL_STABLE_OUTPUT;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


}
