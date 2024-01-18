package frc.robot.subsystems;

import ca.team4308.absolutelib.wrapper.LogSubsystem;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import frc.robot.Constants;

public class LimelightSubsystem extends LogSubsystem {

    public static NetworkTable limelight;
    public static NetworkTable botpose;

    public LimelightSubsystem(){
        limelight = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
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

    public double getLatency() {
        double[] botpose = limelight.getEntry("botpose").getDoubleArray(new double[] {});
        return botpose[6];
    }

    public double getXPose() {
        double[] botpose = limelight.getEntry("botpose").getDoubleArray(new double[] {});
        return botpose[0];
    }

    public double getYPose() {
        double[] botpose = limelight.getEntry("botpose").getDoubleArray(new double[] {});
        return botpose[1];
    }

    @Override
    public Sendable log() {
        return this;
    }
}