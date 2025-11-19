// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Turret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Shoot extends Command {
  private final Turret turret;
  private final Camera camera;
  
  private final PIDController horizontalPID = new PIDController(0, 0, 0);
  private final PIDController verticalPID = new PIDController(0, 0, 0);

  private boolean failed = false;

  /** Creates a new Shoot. */
  public Shoot() {
    turret = Turret.getInstance();
    camera = Camera.getInstance();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  private double estimateTravelTime(double distance) {
    return distance/10.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double[] angle = camera.relativeAngleToTarget();

    if (angle.length == 0) {
      failed = true;
      return;
    }

    double horizontal = angle[0];
    double vertical = angle[1];

    double distance = camera.distToClosestTarget();
    double travelTime = estimateTravelTime(distance);

    double[] position_estimate = {
      Math.sin(horizontal)*Math.cos(vertical)*distance,
      Math.cos(horizontal)*Math.cos(vertical)*distance,
      Math.sin(vertical)*distance
    };
    // x right y forward, z up

    double[] velocity = camera.getClosestTargetVelocity();
    position_estimate[0] += velocity[0]*travelTime;
    position_estimate[2] += velocity[1]*travelTime;

    // TODO: Gravity Estimate, feed error into PID, targeting 0.

    turret.setSpeeds(
      horizontalPID.calculate(, 0),
      verticalPID.calculate(, 0)
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.setSpeeds(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return failed;
  }
}
