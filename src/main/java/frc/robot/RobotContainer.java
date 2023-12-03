package frc.robot;

import java.util.ArrayList;

import ca.team4308.absolutelib.control.JoystickHelper;
import ca.team4308.absolutelib.control.XBoxWrapper;
import ca.team4308.absolutelib.math.DoubleUtils;
import ca.team4308.absolutelib.math.Vector2;
import ca.team4308.absolutelib.wrapper.LogSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
    public final ArrayList<LogSubsystem> subsystems = new ArrayList<LogSubsystem>();

    // Subsystems
    private final DriveSubsystem m_driveSubsystem;

    // Commands
    private final DriveCommand driveCommand;
    
    // Controllers
    public final XBoxWrapper stick = new XBoxWrapper(0);
    public final XBoxWrapper stick2 = new XBoxWrapper(1);

    public RobotContainer() {
        m_driveSubsystem = new DriveSubsystem();
        subsystems.add(m_driveSubsystem);

        driveCommand = new DriveCommand(m_driveSubsystem, () -> getDriveControl());
        m_driveSubsystem.setDefaultCommand(driveCommand);

    }

    public Vector2 getDriveControl() {
        double drive = DoubleUtils.normalize(stick.getLeftY());

        double turn = DoubleUtils.normalize(stick.getRightX());

        Vector2 control = new Vector2(drive, turn);
        control = JoystickHelper.ScaledAxialDeadzone(control, 0.14);
        control = JoystickHelper.scaleStick(control, 2);
        control = JoystickHelper.clampStick(control);
        return control;
    }

    public Command getAutonomousCommand() {
        return driveCommand;
    }
}
