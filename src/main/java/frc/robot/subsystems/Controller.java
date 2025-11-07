// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Controller extends SubsystemBase {
  /** Creates a new Controller. */
  private final CommandXboxController m_driverController =
      new CommandXboxController(0);

  public enum Stick {
    Left,
    Right
  } 
  public enum Axis {
    X,
    Y 
  } 
  static Controller _instance;
  public static Controller getInstance(){
    if (_instance == null){
      _instance = new Controller(); 
    }
    return _instance;
  }
  

  public Controller() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public double getStickValue(Stick stick, Axis axis) {
    if(stick == Stick.Left) {
      if(axis == Axis.X) {
        return m_driverController.getLeftX(); 
      } else {
        return m_driverController.getLeftY(); 
      }
    } else {
      if(axis == Axis.X) {
        return m_driverController.getRightX(); 
      } else {
        return m_driverController.getRightY(); 
      }

    }
  }
}
