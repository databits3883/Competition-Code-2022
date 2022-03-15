// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  private static NetworkTable table;

  private pipeline desiredPipeline;

  private boolean pipelineSetLazy = false;

  public Vision() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    desiredPipeline = pipeline.driverCam;
    
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
    boolean set = table.getEntry("getpipe").getNumber(-1).intValue() == desiredPipeline.pipeNumber;
    pipelineSetLazy = set;
    return set;
  }
  public boolean getPipelineSetLazy(){
    return pipelineSetLazy? true:pipelineSet();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public enum pipeline{
    driverCam(0),
    hub(1),
    hubAlternate(9);

    public final int pipeNumber;

    pipeline(int n){
      pipeNumber = n;
    }
  }
}