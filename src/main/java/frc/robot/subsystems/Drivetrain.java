// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  WPI_TalonSRX leftFollower = new WPI_TalonSRX(Constants.CanIDs.LEFT_FOLLOWER);
  WPI_TalonSRX leftLeader = new WPI_TalonSRX(Constants.CanIDs.LEFT_LEADER);

  WPI_TalonSRX rightFollower = new WPI_TalonSRX(Constants.CanIDs.RIGHT_FOLLOWER);
  WPI_TalonSRX rightLeader = new WPI_TalonSRX(Constants.CanIDs.RIGHT_LEADER);
  
  MotorController leftController; 
  
  DifferentialDrive driveObj; 
  
  static Drivetrain _instance;
  public static Drivetrain getInstance() {
    if(_instance == null) {
      _instance = new Drivetrain();
    }
    return _instance;
  }

  /** Creates a new Drive. */
  public Drivetrain() {
    leftLeader.setNeutralMode(NeutralMode.Brake); 
    rightLeader.setNeutralMode(NeutralMode.Brake); 

    leftFollower.follow(leftLeader);
    rightLeader.follow(rightLeader);
    
    driveObj = new DifferentialDrive(leftLeader, rightLeader);
  }

  @Override
  public void periodic() {
    
  }
  public void drive(double drive, double turn) {
    driveObj.arcadeDrive(drive, turn);
  }
}
