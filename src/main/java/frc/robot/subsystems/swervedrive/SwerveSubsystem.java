// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import java.io.File;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {

  private final SwerveDrive swerveDrive;

  public double maximumSpeed = 4.60248;

  private SwerveAutoBuilder autoBuilder = null;

  public SwerveSubsystem(File directory) {

    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    try {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
    swerveDrive.setHeadingCorrection(true); 
    // Heading correction should only be used while controlling the robot via angle.

  }

  public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg) {
    swerveDrive = new SwerveDrive(driveCfg, controllerCfg, maximumSpeed);
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    swerveDrive.drive(translation, rotation, fieldRelative, false);
    // Open loop is disabled since it shouldn't be used most of the time.
  }

  public void driveFieldOriented(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
  }

  public void drive(ChassisSpeeds velocity) {
    swerveDrive.drive(velocity);
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }

  public SwerveDriveKinematics getKinematics() {
    return swerveDrive.kinematics;
  }

  public void resetOdometry(Pose2d initialHolonomicPose) {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  public void postTrajectory(Trajectory trajectory) {
    swerveDrive.postTrajectory(trajectory);
  }

  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  public void setMotorBrake(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }

  public Rotation2d getHeading() {
    return swerveDrive.getYaw();
  }

  /**
   * Get the chassis speeds based on controller input of 2 joysticks. One for
   * speeds in which direction. The other for
   * the angle of the robot.
   *
   * @param xInput   X joystick input for the robot to move in the X direction.
   * @param yInput   Y joystick input for the robot to move in the Y direction.
   * @param headingX X joystick which controls the angle of the robot.
   * @param headingY Y joystick which controls the angle of the robot.
   * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
   */

  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
    xInput = Math.pow(xInput, 3);
    yInput = Math.pow(yInput, 3);
    return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, headingX, headingY, getHeading().getRadians(),
        maximumSpeed);
  }

  /**
   * Get the chassis speeds based on controller input of 1 joystick and one angle.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param angle  The angle in as a {@link Rotation2d}.
   * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
    xInput = Math.pow(xInput, 3);
    yInput = Math.pow(yInput, 3);
    return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, angle.getRadians(), getHeading().getRadians(),
        maximumSpeed);
  }

  public ChassisSpeeds getFieldVelocity() {
    return swerveDrive.getFieldVelocity();
  }

  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  public SwerveController getSwerveController() {
    return swerveDrive.swerveController;
  }

  public SwerveDriveConfiguration getSwerveDriveConfiguration() {
    return swerveDrive.swerveDriveConfiguration;
  }

  // Lock the swerve drive to prevent it from moving.
  public void lock() {
    swerveDrive.lockPose();
  }

  public Rotation2d getPitch() {
    return swerveDrive.getPitch();
  }

  public void addFakeVisionReading() {
    swerveDrive.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp());
  }

  /**
   * Factory to fetch the PathPlanner command to follow the defined path.
   *
   * @param path             Path planner path to specify.
   * @param constraints      {@link PathConstraints} for
   *                         {@link com.pathplanner.lib.PathPlanner#loadPathGroup}
   *                         function
   *                         limiting velocity and acceleration.
   * @param eventMap         {@link java.util.HashMap} of commands corresponding
   *                         to path planner events given as
   *                         strings.
   * @param translation      The {@link PIDConstants} for the translation of the
   *                         robot while following the path.
   * @param rotation         The {@link PIDConstants} for the rotation of the
   *                         robot while following the path.
   * @param useAllianceColor Automatically transform the path based on alliance
   *                         color.
   * @return PathPlanner command to follow the given path.
   */
  public Command creatPathPlannerCommand(String path, PathConstraints constraints, Map<String, Command> eventMap,
      PIDConstants translation, PIDConstants rotation, boolean useAllianceColor) {
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(path, constraints);
    // SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
    // Pose2d supplier,
    // Pose2d consumer- used to reset odometry at the beginning of auto,
    // PID constants to correct for translation error (used to create the X and Y
    // PID controllers),
    // PID constants to correct for rotation error (used to create the rotation
    // controller),
    // Module states consumer used to output to the drive subsystem,
    // Should the path be automatically mirrored depending on alliance color.
    // Optional- defaults to true
    // )
    if (autoBuilder == null) {
      autoBuilder = new SwerveAutoBuilder(
          swerveDrive::getPose,
          swerveDrive::resetOdometry,
          translation,
          rotation,
          swerveDrive::setChassisSpeeds,
          eventMap,
          useAllianceColor,
          this);
    }

    return autoBuilder.fullAuto(pathGroup);
  }
}
