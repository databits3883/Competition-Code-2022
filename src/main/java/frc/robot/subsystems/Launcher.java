// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.robot.Constants.LauncherContants.*;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase {
  private final CANSparkMax m_primaryMotor;
  private final CANSparkMax m_secondaryMotor;
  private final SparkMaxPIDController m_primaryController;
  private final SparkMaxPIDController m_secondaryController;
  
  private final RelativeEncoder m_primaryEncoder;
  private final RelativeEncoder m_secondaryEncoder;

  /*public double setpointCopy = 0;
  double pcopy=0;
  double icopy=0;
  double dcopy=0;
  double fcopy=0;*/

  /** Creates a new Launcher. */
  public Launcher() {
    m_primaryMotor = new CANSparkMax(LEADER_CHANNEL, MotorType.kBrushless);
    m_secondaryMotor = new CANSparkMax(FOLLOWER_CHANNEL, MotorType.kBrushless);

    m_secondaryMotor.setInverted(false);
    m_primaryMotor.setInverted(false);
    
    m_primaryMotor.setIdleMode(IdleMode.kCoast);
    m_secondaryMotor.setIdleMode(IdleMode.kCoast);

    m_primaryController = m_primaryMotor.getPIDController();
    m_secondaryController = m_secondaryMotor.getPIDController();
    m_primaryEncoder = m_primaryMotor.getEncoder();
    m_secondaryEncoder = m_primaryMotor.getEncoder();


    m_primaryEncoder.setVelocityConversionFactor(ENCODER_POSITIONAL_CONVERSION);
    m_secondaryEncoder.setVelocityConversionFactor(ENCODER_POSITIONAL_CONVERSION);


    m_primaryController.setFF(0.0003);
    m_primaryController.setP(0.001);
    m_primaryController.setI(0);
    m_primaryController.setD(0);

    m_secondaryController.setFF(0.0003);
    m_secondaryController.setP(0.001);
    m_secondaryController.setI(0);
    m_secondaryController.setD(0);
  }

  public void SetShooterSpeed(double rpm){
    m_primaryController.setReference(-rpm, ControlType.kVelocity);
    m_secondaryController.setReference(-rpm/PRIMARY_SECONDARY_RATIO, ControlType.kVelocity);
    //setpointCopy = rpm;
  }

  public void setDutyCycle(double dutyCycle){
    m_primaryMotor.set(-dutyCycle);
    m_secondaryMotor.set(-dutyCycle/PRIMARY_SECONDARY_RATIO);
  }

  /*@Override 
  public void initSendable(SendableBuilder builder){
    super.initSendable(builder);
    builder.addDoubleProperty("kp", ()->pcopy,(p)->{ m_controller.setP(p); pcopy=p;});
    builder.addDoubleProperty("ki",()->icopy, (i)->{ m_controller.setI(i); icopy=i;});
    builder.addDoubleProperty("kd", ()->dcopy, (d)->{ m_controller.setD(d); dcopy=d;});
    builder.addDoubleProperty("kf", ()->fcopy, (f)->{ m_controller.setFF(f); fcopy=f;});

    builder.addDoubleProperty("setpoint",()->this.setpointCopy, this::SetShooterSpeed);
    builder.addDoubleProperty("speed", m_encoder::getVelocity, null);

  }*/

  
}
