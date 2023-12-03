package frc.robot.subsystems;

import java.util.ArrayList;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import ca.team4308.absolutelib.wrapper.LogSubsystem;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.util.sendable.Sendable;

public class DriveSubsystem extends LogSubsystem {

    private final CANSparkMax frontLeftDriveMotor;
    private final CANSparkMax backLeftDriveMotor;
    private final CANSparkMax frontRightDriveMotor;
    private final CANSparkMax backRightDriveMotor;

    private final CANSparkMax frontLeftTurningMotor;
    private final CANSparkMax backLeftTurningMotor;
    private final CANSparkMax frontRightTurningMotor;
    private final CANSparkMax backRightTurningMotor;

    private ArrayList<CANSparkMax> controllersSparkMax = new ArrayList<CANSparkMax>();

    public DriveSubsystem() {

        // Setup and Add Controllers
        frontLeftDriveMotor = new CANSparkMax(DriveConstants.kFrontLeftDriveMotorPort, MotorType.kBrushless);
        backLeftDriveMotor = new CANSparkMax(DriveConstants.kBackLeftDriveMotorPort, MotorType.kBrushless);
        frontRightDriveMotor = new CANSparkMax(DriveConstants.kFrontRightDriveMotorPort, MotorType.kBrushless);
        backRightDriveMotor = new CANSparkMax(DriveConstants.kBackRightDriveMotorPort, MotorType.kBrushless);
        controllersSparkMax.add(frontLeftDriveMotor);
        controllersSparkMax.add(backLeftDriveMotor);
        controllersSparkMax.add(frontRightDriveMotor);
        controllersSparkMax.add(backRightDriveMotor);

        frontLeftTurningMotor = new CANSparkMax(DriveConstants.kFrontLeftTurningMotorPort, MotorType.kBrushless);
        backLeftTurningMotor = new CANSparkMax(DriveConstants.kBackLeftTurningMotorPort, MotorType.kBrushless);
        frontRightTurningMotor = new CANSparkMax(DriveConstants.kFrontRightTurningMotorPort, MotorType.kBrushless);
        backRightTurningMotor = new CANSparkMax(DriveConstants.kBackRightTurningMotorPort, MotorType.kBrushless);
        controllersSparkMax.add(frontLeftTurningMotor);
        controllersSparkMax.add(backLeftTurningMotor);
        controllersSparkMax.add(frontRightTurningMotor);
        controllersSparkMax.add(backRightTurningMotor);

        // Set Follow
        backLeftDriveMotor.follow(frontLeftDriveMotor);
        frontRightDriveMotor.follow(frontLeftDriveMotor);
        backRightDriveMotor.follow(frontLeftDriveMotor);

        backLeftTurningMotor.follow(frontLeftTurningMotor);
        frontRightTurningMotor.follow(frontLeftTurningMotor);
        backRightTurningMotor.follow(frontLeftTurningMotor);

        // Change Config For All Controllers        
        for (CANSparkMax spark : controllersSparkMax) {
            spark.restoreFactoryDefaults();
        }

        // Reset
        stopControllers();
    }
    
    public void setMotorOutput(double drive, double turn) {
        frontLeftDriveMotor.set(drive);
        frontLeftTurningMotor.set(turn);
    }

    public void stopControllers() {
        frontLeftDriveMotor.set(0);
        frontLeftTurningMotor.set(0);
    }

    @Override
    public Sendable log() {
        return this;
    }
}
