// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import ca.team4308.absolutelib.control.XBoxWrapper;
import ca.team4308.absolutelib.control.JoystickHelper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;

import frc.robot.commands.swervedrive.auto.Autos;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDrive;
import frc.robot.commands.swervedrive.drivebase.AbsoluteFieldDrive;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class RobotContainer {

  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  public final XBoxWrapper stick = new XBoxWrapper(0);
  public final XBoxWrapper stick2 = new XBoxWrapper(1);

  public final AbsoluteDrive closedAbsoluteDrive;
  public final TeleopDrive closedFieldRel;


  public RobotContainer() {
    configureBindings();

     closedAbsoluteDrive = new AbsoluteDrive(drivebase,
        // Applies deadbands and inverts controls because joysticks
        // are back-right positive while robot
        // controls are front-left positive
        () -> MathUtil.applyDeadband(-stick.getLeftY(),
            OperatorConstants.kInputDeadband),
        () -> MathUtil.applyDeadband(-stick.getLeftX(),
            OperatorConstants.kInputDeadband),
        () -> -stick.getRightX(),
        () -> -stick.getRightY());

    AbsoluteFieldDrive closedFieldAbsoluteDrive = new AbsoluteFieldDrive(drivebase,
        () -> MathUtil.applyDeadband(stick.getLeftY(), OperatorConstants.kInputDeadband),
        () -> MathUtil.applyDeadband(stick.getLeftX(), OperatorConstants.kInputDeadband),
        () -> stick.getRightX());

    TeleopDrive simClosedFieldRel = new TeleopDrive(drivebase,
        () -> MathUtil.applyDeadband(stick.getLeftY(), OperatorConstants.kInputDeadband),
        () -> MathUtil.applyDeadband(stick.getLeftX(), OperatorConstants.kInputDeadband),
        () -> stick.getRightX(), () -> setFieldMode());

     closedFieldRel = new TeleopDrive(drivebase,
        () -> MathUtil.applyDeadband(stick.getLeftY(), OperatorConstants.kInputDeadband),
        () -> MathUtil.applyDeadband(stick.getLeftX(), OperatorConstants.kInputDeadband),
        () -> -stick.getRightX(), () -> setFieldMode());

      drivebase.setDefaultCommand(closedAbsoluteDrive);

  }

  private void configureBindings() {

    stick.A.onTrue((new InstantCommand(drivebase::zeroGyro)));
    stick.X.onTrue(new InstantCommand(drivebase::addFakeVisionReading));
    stick.B.whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(drivebase);
  }

  public boolean setFieldMode() {
    return true;
  }

  public void setDriveMode() {
    if (stick.LB.getAsBoolean()) {
      drivebase.setDefaultCommand(closedFieldRel);
    } else if (stick.RB.getAsBoolean()) {
      drivebase.setDefaultCommand(closedAbsoluteDrive);
    }

  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
