package frc.robot.subsystems;

import ca.team4308.absolutelib.wrapper.LogSubsystem;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimelightSystem extends LogSubsystem {

    public static NetworkTable limelight;

    public LimelightSystem(){
        limelight = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    }

    public double getXAngle() {
        return limelight.getEntry("tx").getDouble(0.0);
    }

    @Override
    public Sendable log() {
        return this;
    }
}
