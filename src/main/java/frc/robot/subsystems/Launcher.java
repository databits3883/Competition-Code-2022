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

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase {
  private final CANSparkMax m_launchMotor;
  private final SparkMaxPIDController m_controller;

  private final CANSparkMax m_secondary;
  private final SparkMaxPIDController m_secondaryController;
  
  private final RelativeEncoder m_encoder;
  private final RelativeEncoder m_secondaryEncoder;

  /** Creates a new Launcher. */
  public Launcher() {
    m_launchMotor = new CANSparkMax(LEADER_CHANNEL, MotorType.kBrushless);

    m_secondary = new CANSparkMax(FOLLOWER_CHANNEL, MotorType.kBrushless);

    m_launchMotor.setIdleMode(IdleMode.kCoast);
    m_secondary.setIdleMode(IdleMode.kCoast);
    

    m_controller = m_launchMotor.getPIDController();
    m_encoder = m_launchMotor.getEncoder();

    m_secondaryController = m_secondary.getPIDController();
    m_secondaryEncoder = m_secondary.getEncoder();

    m_encoder.setVelocityConversionFactor(ENCODER_POSITIONAL_CONVERSION);

    m_controller.setP(0.0001);
    m_controller.setI(0);
    m_controller.setD(0);
    m_controller.setFF(0.00027);

    m_secondaryController.setP(0.0003);
    m_secondaryController.setI(0);
    m_secondaryController.setD(0);
    m_secondaryController.setFF(0.00009);
  }

  public void SetShooterSpeed(double rpm){
    m_controller.setReference(rpm, ControlType.kVelocity);
    m_secondaryController.setReference(rpm/2, ControlType.kVelocity);
  }

  public void setDutyCycle(double dutyCycle){
    m_launchMotor.set(dutyCycle);
    m_secondary.set(dutyCycle/2);
  }

  // @Override 
  // public void initSendable(SendableBuilder builder){
  //   super.initSendable(builder);
  //   NetworkTable table = NetworkTableInstance.getDefault().getTable("launcher tuning");
  //   table.getEntry("mainP").setNumber(m_controller.getP());
  //   table.addEntryListener("mainP", (t,string,entry,value,i)->m_controller.setP(value.getDouble()), EntryListenerFlags.kUpdate);

  //   table.getEntry("mainI").setNumber(m_controller.getI());
  //   table.addEntryListener("mainI", (t,string,entry,value,i)->m_controller.setI(value.getDouble()), EntryListenerFlags.kUpdate);

  //   table.getEntry("mainD").setNumber(m_controller.getD());
  //   table.addEntryListener("mainD", (t,string,entry,value,i)->m_controller.setD(value.getDouble()), EntryListenerFlags.kUpdate);

  //   table.getEntry("mainFF").setNumber(m_controller.getFF());
  //   table.addEntryListener("mainFF", (t,string,entry,value,i)->m_controller.setFF(value.getDouble()), EntryListenerFlags.kUpdate);
  
  
  //   table.getEntry("secondaryP").setNumber(m_secondaryController.getP());
  //   table.addEntryListener("secondaryP", (t,string,entry,value,i)->m_secondaryController.setP(value.getDouble()), EntryListenerFlags.kUpdate);

  //   table.getEntry("secondaryI").setNumber(m_secondaryController.getI());
  //   table.addEntryListener("secondaryI", (t,string,entry,value,i)->m_secondaryController.setI(value.getDouble()), EntryListenerFlags.kUpdate);

  //   table.getEntry("secondaryD").setNumber(m_secondaryController.getD());
  //   table.addEntryListener("secondaryD", (t,string,entry,value,i)->m_secondaryController.setD(value.getDouble()), EntryListenerFlags.kUpdate);

  //   table.getEntry("secondaryFF").setNumber(m_secondaryController.getFF());
  //   table.addEntryListener("secondaryFF", (t,string,entry,value,i)->m_secondaryController.setFF(value.getDouble()), EntryListenerFlags.kUpdate);



  //   table.getEntry("mainSetpoint").setNumber(0);
  //   table.addEntryListener("mainSetpoint", (t,string,entry,value,i)->m_controller.setReference(value.getDouble(),ControlType.kVelocity), EntryListenerFlags.kUpdate);

  //   table.getEntry("secondarySetpoint").setNumber(0);
  //   table.addEntryListener("secondarySetpoint", (t,string,entry,value,i)->m_secondaryController.setReference(value.getDouble(),ControlType.kVelocity), EntryListenerFlags.kUpdate);

  //   builder.addDoubleProperty("main speed", m_encoder::getVelocity, null);
  //   builder.addDoubleProperty("secondary speed", m_secondaryEncoder::getVelocity, null);

  // }

  
}
