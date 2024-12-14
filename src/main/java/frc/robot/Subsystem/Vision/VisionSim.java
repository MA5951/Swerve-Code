package frc.robot.Subsystem.Vision;

import java.util.List;
import java.util.stream.Collectors;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonTargetSortMode;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ma5951.utils.Utils.GeomUtil;
import com.ma5951.utils.Vision.Limelights.LimelightHelpers.PoseEstimate;
import com.ma5951.utils.Vision.Limelights.LimelightHelpers.RawDetection;
import com.ma5951.utils.Vision.Limelights.LimelightHelpers.RawFiducial;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Subsystem.Swerve.SwerveConstants;

public class VisionSim implements VisionIO {

    private VisionSystemSim visionSim;
    private AprilTagFieldLayout tagLayout;
    private SimCameraProperties cameraProp;
    private PhotonCamera camera;
    private PhotonCameraSim cameraSim;
    private PhotonPipelineResult result;
    private int numOftags = 0;

    public VisionSim() {
        visionSim = new VisionSystemSim("main");
        try {
            tagLayout = AprilTagFieldLayout
                    .loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);

        } catch (Exception e) {
        }

        visionSim.addAprilTags(tagLayout);

        cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(1280, 800, Rotation2d.fromDegrees(86));
        cameraProp.setFPS(18);
        cameraProp.setCalibError(0.6, 0.2);
        cameraProp.setAvgLatencyMs(30);
        cameraProp.setLatencyStdDevMs(5);

        camera = new PhotonCamera("SimCam");
        cameraSim = new PhotonCameraSim(camera, cameraProp);

        cameraSim.setTargetSortMode(PhotonTargetSortMode.Largest);
        cameraSim.setMaxSightRange(5);

        visionSim.addCamera(cameraSim, VisionConstants.robotToCamera);
        visionSim.update(new Pose2d(2, 2, new Rotation2d()));

        cameraSim.enableRawStream(true);
        cameraSim.enableProcessedStream(true);
        //cameraSim.setWireframeResolution(1280);
        cameraSim.enableDrawWireframe(true);

        
    }

    @Override
    public PoseEstimate getEstimatedPose() {
        if (result.getMultiTagResult().isPresent()) {

            return new PoseEstimate(
                    GeomUtil.toPose2d(result.getMultiTagResult().get().estimatedPose.best),
                    result.getTimestampSeconds(),
                    Timer.getFPGATimestamp() - result.getTimestampSeconds(), getTargetCount(), 1, 0, 0, null);
        }

        return new PoseEstimate();
    }

    @Override
    public boolean isTarget() {
        if (numOftags != 0) {
            return result.hasTargets();
        }
        return false;
    }

    @Override
    public RawDetection getRawDetection(int detectionIndex) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getRawDetection'");
    }

    @Override
    public RawDetection getRawDetection() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getRawDetection'");
    }

    @Override
    public RawFiducial getRawFiducial(int detectionIndex) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getRawFiducial'");
    }

    @Override
    public RawFiducial getRawFiducial() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getRawFiducial'");
    }

    @Override
    public void filterTags(int[] tagsArry) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'filterTags'");
    }

    @Override
    public double getTx() {
        if (numOftags != 0) {
            return result.getBestTarget().getYaw();
        }
        return 0d;
        
    }

    @Override
    public double getTy() {
        if (numOftags != 0) {
            return result.getBestTarget().getPitch();
        }
        return 0d;
    }

    @Override
    public double getTa() {
        if (numOftags != 0) {
            return result.getBestTarget().getArea();
        }
        return 0d;
    }

    @Override
    public int getTargetCount() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getTargetCount'");
    }

    @Override
    public int getTagID() {
        return result.targets.get(0).getFiducialId();
    }

    public static Pose3d pose2dToPose3d(Pose2d pose) {
        return new Pose3d(
                pose.getX(), pose.getY(), 0, new Rotation3d(0, 0, pose.getRotation().getRadians()));
    }

    public void update() {
        PhotonPipelineResult latestResult = cameraSim.process(
                0,
                GeomUtil.toPose3d((SwerveConstants.SWERVE_DRIVE_SIMULATION.getSimulatedDriveTrainPose()))
                        .plus(VisionConstants.robotToCamera),
                tagLayout.getTags().stream()
                        .map(
                                (a) -> new VisionTargetSim(
                                        a.pose, TargetModel.kAprilTag36h11, a.ID))
                        .collect(Collectors.toList()));
        cameraSim.submitProcessedFrame(latestResult);
        visionSim.update(SwerveConstants.SWERVE_DRIVE_SIMULATION.getSimulatedDriveTrainPose());
        List<PhotonPipelineResult> latest = camera.getAllUnreadResults();
        numOftags = latest.size();
        System.out.println(numOftags);
        if (latest.size() > 0) {
            result = latest.get(0);
        } else {
            result = new PhotonPipelineResult();
        }
    }

}
