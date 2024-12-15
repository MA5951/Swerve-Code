package frc.robot.Subsystem.Vision;

import java.io.Console;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonTargetSortMode;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
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
    private boolean isResult = true;
    private PhotonPoseEstimator poseEstimator;
    private List<Integer> lastFilterArry;
    private PhotonPipelineResult blankResult = new PhotonPipelineResult();;

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

        poseEstimator = new PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                VisionConstants.robotToCamera);

        cameraSim.enableRawStream(true);
        cameraSim.enableProcessedStream(true);
        // cameraSim.setWireframeResolution(1280);
        cameraSim.enableDrawWireframe(true);

        // filterTags(new int[] {0});

    }

    @Override
    public PoseEstimate getEstimatedPose() {
        System.out.println(poseEstimator.update(result).get().estimatedPose.toPose2d().toString());
        if (poseEstimator.update(result).isPresent() ) {

            return new PoseEstimate(
                    poseEstimator.update(result).get().estimatedPose.toPose2d(),
                    result.getTimestampSeconds(),
                    Timer.getFPGATimestamp() - result.getTimestampSeconds(), getTargetCount(), 1, 0, 0, null);
        }

        return new PoseEstimate();
    }

    @Override
    public boolean isTarget() {
        return result.hasTargets();
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
        // lastFilterArry = new ArrayList<Integer>(tagsArry.length);
        // for (int i : tagsArry) {
        // lastFilterArry.add(i);
        // }
    }

    @Override
    public double getTx() {
        return result.getBestTarget().yaw;

    }

    @Override
    public double getTy() {
        return result.getBestTarget().pitch;
    }

    @Override
    public double getTa() {
        return result.getBestTarget().area;
    }

    @Override
    public int getTargetCount() {
        return result.targets.size();
    }

    @Override
    public int getTagID() {
        return result.targets.get(0).getFiducialId();
    }

    private PhotonPipelineResult filterTagsID() {

        for (PhotonTrackedTarget tag : result.targets) {
            if (lastFilterArry.contains(Integer.valueOf(tag.fiducialId))) {
                result.targets.remove(tag);
            }
        }

        return result;
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
        result = latest.get(0);
        // filterTagsID();
    }

}
