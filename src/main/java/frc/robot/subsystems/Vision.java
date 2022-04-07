// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  private static NetworkTable table;

  private pipeline desiredPipeline;

  private boolean pipelineSetLazy = false;

  private Servo limelightServo = new Servo(Constants.VisionConstants.LIMELIGHT_SERVO_PWM_CHANNEL);

  public Vision() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    desiredPipeline = pipeline.driverCam;
    Shuffleboard.getTab("tab 6").addNumber("distance in feet", this::getDistanceVision);
    
  }
  public double getHorizontalOffset(){
    return table.getEntry("tx").getDouble(0);
  }

  public boolean isTargetValid(){
    if (table.getEntry("tv").getNumber(0).intValue() == 1){
      return true;
    }
    else{
      return false;
    }
  }

  public void setMode(pipeline p){
    table.getEntry("pipeline").setNumber(p.pipeNumber);
    pipelineSetLazy = false;
  }
  public boolean pipelineSet(){
    boolean set = table.getEntry("getpipe").getNumber(-1).equals(desiredPipeline.pipeNumber);
    pipelineSetLazy = set;
    return set;
  }
  public boolean getPipelineSetLazy(){
    //return pipelineSetLazy? true:pipelineSet();
    //TODO:fix
    return true;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    lockServo();
    //System.out.println(getDistanceVision(getAngleOfElevation()));

  }


  public void lockServo(){
    limelightServo.set(0.73);
  }

  public double getDistanceVision(){
    return ((Constants.VisionConstants.TARGET_HEIGHT_FEET-2.0833)/Math.tan(Math.toRadians(45 + table.getEntry("ty").getDouble(0)))+1.0);
  }
  public double getShooterSpeedFromDistance(double distance, boolean isTargetValid){
    
    double setSpeed = 86.7*distance + 1318;

    double correctedSpeed = Math.min(1900, Math.max(1500, setSpeed));
    
    if(isTargetValid) return correctedSpeed;
    else{return 0;}

  }

  

  public enum pipeline{
    driverCam(0),
    hub(1),
    hubAlternate(4);

    public final Number pipeNumber;

    pipeline(Number n){
      pipeNumber = n;
    }
  }
}