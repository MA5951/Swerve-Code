package frc.robot.Subsystem.Vision;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import com.ma5951.utils.Vision.Limelights.LimelightHelpers.PoseEstimate;
import com.ma5951.utils.Vision.Limelights.LimelightHelpers.RawDetection;
import com.ma5951.utils.Vision.Limelights.LimelightHelpers.RawFiducial;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Subsystem.Swerve.SwerveConstants;

public class VisionSim implements VisionIO {

    private VisionSystemSim visionSim;
    private AprilTagFieldLayout tagLayout;
    private SimCameraProperties cameraProp;
    private PhotonCamera camera;
    private PhotonCameraSim cameraSim;

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

        cameraSim.enableRawStream(true);
        cameraSim.enableProcessedStream(true);
        cameraSim.enableDrawWireframe(true);

        visionSim.addCamera(cameraSim, VisionCOnstants.robotToCamera);
    }

    @Override
    public PoseEstimate getEstimatedPose() {
        return new PoseEstimate(null, getTy(), getTx(), getTargetCount(), getTargetCount(), getTagID(), getTa(), null);
    }

    @Override
    public boolean isTarget() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'isTarget'");
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
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getTx'");
    }

    @Override
    public double getTy() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getTy'");
    }

    @Override
    public double getTa() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getTa'");
    }

    @Override
    public int getTargetCount() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getTargetCount'");
    }

    @Override
    public int getTagID() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getTagID'");
    }

    public void update() {
        visionSim.update(SwerveConstants.SWERVE_DRIVE_SIMULATION.getSimulatedDriveTrainPose());
    }

}
