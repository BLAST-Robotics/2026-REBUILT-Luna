package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {
    private static VisionSubsystem instance;

    public static VisionSubsystem getInstance() {
        if (instance == null) {
            instance = new VisionSubsystem();
        }
        return instance;
    }

    private VisionSubsystem() {
    }

    public static class VisionResult {
        public Pose2d pose;
        public double timestamp;
        public int tagCount;

        public VisionResult(Pose2d pose, double timestamp, int tagCount) {
            this.pose = pose;
            this.timestamp = timestamp;
            this.tagCount = tagCount;
        }
    }

    public VisionResult getVisionResult() {
        try {
            // Use the standard botpose (MegaTag1 / tag-based) estimation instead of MegaTag2.
            // MegaTag1 publishes the "botpose_wpiblue" entry and does not require calling SetRobotOrientation().
            // If you need gyro-assisted constraints in the future, re-enable orientation updates carefully.
            var estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-four");
            
            if (estimate == null || estimate.pose == null) {
                return new VisionResult(new Pose2d(), 0, 0);
            }
            
            return new VisionResult(
                estimate.pose, 
                estimate.timestampSeconds, 
                estimate.tagCount
            );
        } catch (Exception e) {
            return new VisionResult(new Pose2d(), 0, 0);
        }
    }

    @Override
    public void periodic() {
        // Safe updates can be performed here later if needed
    }
}
