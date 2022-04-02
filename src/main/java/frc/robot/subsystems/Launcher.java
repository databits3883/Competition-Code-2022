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

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase {

  private final CANSparkMax m_launchMotor;
  private final SparkMaxPIDController m_controller;

  private final CANSparkMax m_secondary;
  private final SparkMaxPIDController m_secondaryController;
  
  private final RelativeEncoder m_encoder;
  private final RelativeEncoder m_secondaryEncoder;


  private double ratio = 0.6;

  /** Creates a new Launcher. */
  public Launcher() {

    m_launchMotor = new CANSparkMax(LEADER_CHANNEL, MotorType.kBrushless);
    m_launchMotor.setInverted(true);

    m_secondary = new CANSparkMax(FOLLOWER_CHANNEL, MotorType.kBrushless);
    m_secondary.setInverted(true);

    m_launchMotor.setIdleMode(IdleMode.kCoast);
    m_secondary.setIdleMode(IdleMode.kCoast);
    

    m_controller = m_launchMotor.getPIDController();
    m_encoder = m_launchMotor.getEncoder();
    m_encoder.setVelocityConversionFactor(1);

    m_secondaryController = m_secondary.getPIDController();
    m_secondaryEncoder = m_secondary.getEncoder();
    m_secondaryEncoder.setVelocityConversionFactor(1);


    m_controller.setP(0.0001);
    m_controller.setI(0);
    m_controller.setD(0.0001);
    m_controller.setFF(0.0007);

    m_secondaryController.setP(0.0002);
    m_secondaryController.setI(0);
    m_secondaryController.setD(0.0001);
    m_secondaryController.setFF(0.000085);

    Shuffleboard.getTab("Debug").addNumber("launcher speed", m_encoder::getVelocity);
  }

  public void SetShooterSpeed(double rpm){
    m_controller.setReference(rpm, ControlType.kVelocity);
    m_secondaryController.setReference(rpm/ratio, ControlType.kVelocity);
  }

  public void setDutyCycle(double dutyCycle){
    m_launchMotor.set(dutyCycle);
    m_secondary.set(dutyCycle/ratio);
  }
  public double getSetSpeed(){
    return m_encoder.getVelocity();

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

  //   table.getEntry("ratio").setNumber(ratio);
  //   table.addEntryListener("ratio", (t,string,entry,value,i)->{ratio = value.getDouble();System.out.println(value.getDouble());}, EntryListenerFlags.kUpdate);
  // }

  @Override 
  public void initSendable(SendableBuilder builder){
    NetworkTable table = NetworkTableInstance.getDefault().getTable("launcher tuning");
    table.getEntry("speed").setNumber(0);
    table.addEntryListener("speed", (t,string,entry,value,i)->{SetShooterSpeed(value.getDouble());System.out.println(value.getDouble());}, EntryListenerFlags.kUpdate);
  }

  
}
