// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.lang.Math;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

import ca.team4308.absolutelib.math.DoubleUtils;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * An example command that uses an example subsystem.
 */
public class AimCommand extends Command {

  private final SwerveSubsystem swerve;
  private final DoubleSupplier control;
  private final PIDController angle_controller = new PIDController(Constants.AngleControl.kP, Constants.AngleControl.kI, Constants.AngleControl.kD);

  public AimCommand(SwerveSubsystem swerve, DoubleSupplier control) {
    this.swerve = swerve;
    this.control = control;
    angle_controller.setSetpoint(0.0);
    angle_controller.setTolerance(Constants.AngleControl.kTolerance);

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = DoubleUtils.clamp(angle_controller.calculate(control.getAsDouble()), Math.PI, -Math.PI);

    swerve.drive(new Translation2d(0,0), output, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted){
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished(){
    return false;
  }
}
