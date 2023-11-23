package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;
    private SparkMaxPIDController builtinTurningPidController;

    private final AnalogInput absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    private String moduleName;

    private MechanismLigament2d simTurn;
    private MechanismLigament2d simDirection;

    private MechanismLigament2d simTurn2;
    private MechanismLigament2d simDirection2;


    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed, String name) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new AnalogInput(absoluteEncoderId);

        moduleName = name;
        SmartDashboard.putNumber(moduleName + " ABE Manual", 0);

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);
        
        builtinTurningPidController = turningMotor.getPIDController();

        // Set PID values for the simulated Spark max PID
        builtinTurningPidController.setP(ModuleConstants.kPTurning);
        builtinTurningPidController.setI(ModuleConstants.kITurning);
        builtinTurningPidController.setD(ModuleConstants.kDTurning);
        builtinTurningPidController.setIZone(0.0);
        builtinTurningPidController.setFF(0.0);
        builtinTurningPidController.setOutputRange(-1, 1);
        turningMotor.burnFlash();

        driveMotor.setIdleMode(IdleMode.kBrake);
        turningMotor.setIdleMode(IdleMode.kBrake);

        driveMotor.setSmartCurrentLimit(40);
        turningMotor.setSmartCurrentLimit(20);


        resetEncoders();

        // Create the mechanism 2d canvas and get the root
        Mechanism2d mod = new Mechanism2d(6,6);
        MechanismRoot2d root = mod.getRoot("climber", 3, 3);

        // Add simTurn to the root, add direction to turn, then add it to smart dashboard
        simTurn = root.append(new MechanismLigament2d("Swerve Turn", 2, 1.75));
        simDirection = simTurn.append(new MechanismLigament2d("Wheel direction", 1, 0, 6, new Color8Bit(Color.kPurple)));
        SmartDashboard.putData(moduleName + " commanded Turn", mod);

        // Do the same thing but for the real module state
        Mechanism2d mod2 = new Mechanism2d(6,6);
        MechanismRoot2d root2 = mod2.getRoot("climber2", 3, 3);

        simTurn2 = root2.append(new MechanismLigament2d("Swerve Turn", 2, 1.75));
        simDirection2 = simTurn2.append(new MechanismLigament2d("Wheel direction", 1, 0, 6, new Color8Bit(Color.kPurple)));
        SmartDashboard.putData(moduleName+"  real Turn", mod2);
    
    }
    public void update(){
        SmartDashboard.putNumber(moduleName + "Absolute-Encoder-Rad", getAbsoluteEncoderRad());
       // SmartDashboard.putNumber(moduleName + " Turning Position", getTurningPosition());
      }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        
        simTurn.setAngle(state.angle); // .plus(Rotation2d.fromDegrees(90))
        simDirection.setAngle(state.speedMetersPerSecond>0? 0:180);

        simTurn2.setAngle(getAbsoluteEncoderRad()); // +90
        simDirection2.setAngle(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond >0 ? 0:180);

        SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}
