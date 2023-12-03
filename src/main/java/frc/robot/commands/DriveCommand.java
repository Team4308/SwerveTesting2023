package frc.robot.commands;

import java.util.function.Supplier;
import ca.team4308.absolutelib.math.Vector2;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {
    private final DriveSubsystem m_subsystem;
    private final Supplier<Vector2> control;

    // 
    public DriveCommand(DriveSubsystem subsystem, Supplier<Vector2> control) {
        m_subsystem = subsystem;
        this.control = control;

        addRequirements(m_subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_subsystem.stopControllers();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Vector2 control = this.control.get();

        m_subsystem.setMotorOutput(control.y, control.x);
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.stopControllers();
    }

}
