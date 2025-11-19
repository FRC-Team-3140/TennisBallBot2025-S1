// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {
  private static Turret instance = null;

  WPI_TalonSRX flyWheelMotorL = new WPI_TalonSRX(Constants.CanIDs.FLYWHEEL_LEFT);
  WPI_TalonSRX flyWheelMotorR = new WPI_TalonSRX(Constants.CanIDs.FLYWHEEL_RIGHT);
  WPI_TalonSRX elevationControlMotor = new WPI_TalonSRX(Constants.CanIDs.ELEVATION_CONTROL);
  WPI_TalonSRX rotationalControlMotor = new WPI_TalonSRX(Constants.CanIDs.ROTATION_CONTROL);
  WPI_TalonSRX intakeControlMotor = new WPI_TalonSRX(Constants.CanIDs.INTAKE_CONTROL);

  DutyCycleEncoder elevationEncoder;
  DutyCycleEncoder rotationEncoder;

  private boolean speedMode = false;
  private double horizontalSpeed = 0;
  private double verticalSpeed = 0;

  private double horizontalSetpoint = 0;
  private double verticalSetpoint = 0;

  private PIDController horizontalPID = new PIDController(0, 0, 0);
  private PIDController verticalPID = new PIDController(0, 0, 0);

  public static Turret getInstance() {
    if (instance == null)
      instance = new Turret();
    return instance;
  }

  private Turret() {
    elevationEncoder = new DutyCycleEncoder(Constants.PwmIDs.ELEVATION_ENCODER);
    rotationEncoder = new DutyCycleEncoder(Constants.PwmIDs.ROTATION_ENCODER);
    horizontalPID.enableContinuousInput(-160, 160);
    elevationEncoder.get();
  }

  public void setSpeeds(double horizontal, double vertical) {
    horizontalSpeed = horizontal;
    verticalSpeed = vertical;
    speedMode = true;
  }

  public void setSetpoint(double horizontal, double vertical) {
    horizontalSetpoint = horizontal;
    verticalSetpoint = vertical;
    speedMode = false;
  }

  @Override
  public void periodic() {
    if (speedMode) {
      rotationalControlMotor.set(horizontalSpeed);
      elevationControlMotor.set(verticalSpeed);
    } else {
      rotationalControlMotor.set(horizontalPID.calculate(rotationEncoder.get(), horizontalSetpoint));
      elevationControlMotor.set(verticalPID.calculate(elevationEncoder.get(), verticalSetpoint));
    }
  }
}
