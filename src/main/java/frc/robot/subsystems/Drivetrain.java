// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.trajectory.Pose;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */

  private CANSparkMax leftMotor = new CANSparkMax(DriveConstants.kLeftPort, MotorType.kBrushless);
  private CANSparkMax rightMotor = new CANSparkMax(DriveConstants.kRightPort, MotorType.kBrushless);;

  private DifferentialDrive drive = new DifferentialDrive(leftMotor, rightMotor);

  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;

  private Pose robotPose;

  private double prevPosition = 0.0;
  
  public Drivetrain() {

    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();

    leftMotor.setInverted(DriveConstants.kLeftInvert);
    rightMotor.setInverted(DriveConstants.kRightInvert);

    leftEncoder = leftMotor.getEncoder();
    rightEncoder = rightMotor.getEncoder();

    leftEncoder.setPositionConversionFactor(DriveConstants.kPosFactor);
    rightEncoder.setPositionConversionFactor(DriveConstants.kPosFactor);

    leftEncoder.setVelocityConversionFactor(DriveConstants.kVelFactor);
    rightEncoder.setVelocityConversionFactor(DriveConstants.kVelFactor);


    robotPose = new Pose(0, 0, 0);

    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updatePos();
  }

  public void stop() {
    drive.stopMotor();
  }

  public void arcadeDrive(double fwd, double rot) {
    drive.arcadeDrive(fwd, rot);
  }

  public void resetEncoders() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  public double getLeftPosition() {
    return leftEncoder.getPosition();
  }

  public double getRightPosition() {
    return rightEncoder.getPosition();
  }

  public double getAveragePosition() {
    return (getLeftPosition() + getRightPosition()) / 2.0;
  }

  public double getLeftVelocity() {
    return leftEncoder.getVelocity();
  }

  public double getRightVelocity() {
    return rightEncoder.getVelocity();
  }

  public double getHeading() {
    return -1.0; // to be filled in 
  }

  public void updatePos() {
    double dp = getAveragePosition() - prevPosition;
    double theta = getHeading();

    double dx = dp * Math.cos(theta);
    double dy = dp * Math.sin(theta);

    robotPose.translate(dx, dy);
    robotPose.setHeading(theta);

    prevPosition = getAveragePosition();
  }
  

}
