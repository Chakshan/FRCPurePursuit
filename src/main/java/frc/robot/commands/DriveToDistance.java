// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveToDistance extends CommandBase {
  /** Creates a new DriveToDistance. */

  private Drivetrain drivetrain;
  private double targetDistance;
  private double kP;
  private double tolerance;


  public DriveToDistance(Drivetrain drivetrain, double targetDistance, double kP, double tolerance) {
    
    this.drivetrain = drivetrain;
    this.targetDistance = targetDistance;
    this.kP = kP;
    this.tolerance = tolerance;

    SmartDashboard.putNumber("kP", kP);
    SmartDashboard.putNumber("tolerance", tolerance);
    SmartDashboard.putNumber("targetDistance", targetDistance);
    SmartDashboard.putNumber("dif", 0.0);

    addRequirements(this.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    drivetrain.resetEncoders();
    this.kP = SmartDashboard.getNumber("kP", 0.0);
    this.targetDistance = SmartDashboard.getNumber("targetDistance", 10.0);
    this.tolerance = SmartDashboard.getNumber("tolerance", 0.0);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double dif = targetDistance - drivetrain.getAveragePosition();
    dif = MathUtil.clamp(dif * kP, -0.5, 0.5);
    drivetrain.arcadeDrive(dif, 0);
    SmartDashboard.putNumber("dif", dif);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(drivetrain.getAveragePosition() - targetDistance) < tolerance;
  }
}
