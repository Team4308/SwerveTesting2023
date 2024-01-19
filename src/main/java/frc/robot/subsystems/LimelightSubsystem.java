package frc.robot.subsystems;

import ca.team4308.absolutelib.wrapper.LogSubsystem;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import frc.robot.Constants;

public class LimelightSubsystem extends LogSubsystem {

    public static NetworkTable limelight;

    public LimelightSubsystem(){
        limelight = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public double getXAngle() {
        if (limelight.getEntry("tv").getDouble(0.0) == 0.0) {
            return 50.0;
        } else {
            return limelight.getEntry("tx").getDouble(0.0);
        }
    }

    public double getYAngle() {
        return limelight.getEntry("ty").getDouble(0.0);
    }

    public double getSpeakerDistance() {
        return (Constants.Field.speakerHeight - Constants.Limelight.lensHeight) / Math.tan((Constants.Limelight.angle + getYAngle()) * (Math.PI / 180.0));
    }

    public void setPipeline(double num) {
        limelight.getEntry("pipeline").setNumber(num);
    }

    @Override
    public Sendable log() {
        return this;
    }
}