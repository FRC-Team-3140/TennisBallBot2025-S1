// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {
  WPI_TalonSRX flyWheelMotorL = new WPI_TalonSRX(Constants.CanIDs.FLYWHEEL_LEFT);
  WPI_TalonSRX flyWheelMotorR = new WPI_TalonSRX(Constants.CanIDs.FLYWHEEL_RIGHT);
  WPI_TalonSRX elevationControlMotor = new WPI_TalonSRX(Constants.CanIDs.ELEVATION_CONTROL);
  WPI_TalonSRX rotationalControlMotor = new WPI_TalonSRX(Constants.CanIDs.ROTATION_CONTROL);
  WPI_TalonSRX intakeControlMotor = new WPI_TalonSRX(Constants.CanIDs.INTAKE_CONTROL);
  
  DutyCycleEncoder elevationEncoder;
  DutyCycleEncoder rotationEncoder;
  
   

  public Turret() {
    elevationEncoder = new DutyCycleEncoder(Constants.PwmIDs.ELEVATION_ENCODER);
    rotationEncoder = new DutyCycleEncoder(Constants.PwmIDs.ROTATION_ENCODER);
    elevationEncoder.get();
  }

  @Override
  public void periodic() {
  }
  
  public void moveTurretLeft() {
    if(rotationEncoder.get() > Constants.Limits.Turret.ROTATION_MIN) {
      rotationalControlMotor.set(0.2);
    }
  }
}
