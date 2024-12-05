package com.ma5951.utils.Vision.Sim;
// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package com.ma5951.utils.Vision;

// import java.io.IOException;
// import java.util.Optional;

// import org.photonvision.EstimatedRobotPose;
// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonPoseEstimator;
// import org.photonvision.PhotonPoseEstimator.PoseStrategy;
// import org.photonvision.estimation.TargetModel;
// import org.photonvision.simulation.PhotonCameraSim;
// import org.photonvision.simulation.SimCameraProperties;
// import org.photonvision.simulation.VisionSystemSim;

// import com.ma5951.utils.Logger.LoggedPose2d;
// import com.ma5951.utils.Vision.CameraTypes.CameraType;
// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.apriltag.AprilTagFields;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.geometry.Translation3d;

// /** Add your docs here. */
// public class CameraSim {
//     private AprilTagFieldLayout tagLayout;
//     private VisionSystemSim visionSim;
//     private TargetModel targetModel;
//     private SimCameraProperties cameraProp;
//     private PhotonCamera camera;
//     private PhotonCameraSim cameraSim;
//     private LoggedPose2d robotPosePV;
//     private Transform3d robotToCamera;
//     private Translation3d robotToCameraTrl;
//     private Rotation3d robotToCameraRot;
//     private PhotonPoseEstimator poseEstimator;

//     public CameraSim(CameraType cameraType) {
//         visionSim = new VisionSystemSim("main");
    
//         try {
//         tagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
//         } catch (IOException e) {
//         e.printStackTrace();
//         }

//         visionSim.addAprilTags(tagLayout);
        
//         cameraProp = new SimCameraProperties();
//         cameraProp.setCalibration(cameraType.getWidth(), cameraType.getHeight(), Rotation2d.fromDegrees(cameraType.getFov()));
//         cameraProp.setCalibError(0.25, 0.08);
//         cameraProp.setFPS(20);
//         cameraProp.setAvgLatencyMs(35);
//         cameraProp.setLatencyStdDevMs(5);
        

//         camera = new PhotonCamera("cameraName");
//         cameraSim = new PhotonCameraSim(camera, cameraProp);
//         robotToCameraTrl = new Translation3d(0, 0, 0.5);
//         robotToCameraRot = new Rotation3d(0, Math.toRadians(-15), 0);
//         robotToCamera = new Transform3d(robotToCameraTrl, robotToCameraRot);

//         visionSim.addCamera(cameraSim, robotToCamera);

//         cameraSim.enableRawStream(true);
//         cameraSim.enableProcessedStream(true);
//         cameraSim.enableDrawWireframe(true);

//         poseEstimator = new PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera ,robotToCamera);
        

        
//     }

//     public VisionSystemSim getVisionSim() {
//         return visionSim;
//     }

//     public PhotonCamera getPhotonCamera() {
//         return camera;
//     }

//     public PhotonCameraSim getCameraSim() {
//         return cameraSim;
//     }

//     // public boolean canSeeTag(int id) {
//     //     Pose3d cameraPose;
//     //     if (visionSim.getCameraPose(cameraSim).isPresent() && visionSim.getCameraPose(cameraSim).get() != null) {
//     //         cameraPose = visionSim.getCameraPose(cameraSim).get();
//     //     } else {
//     //         cameraPose = new Pose3d();
//     //     }
        
//     //     return cameraSim.canSeeTargetPose(cameraPose, visionSim.getVisionTargets("apriltag").toArray()[id]);
//     // }

//     public Optional<EstimatedRobotPose> getEstiPose() {
//         return poseEstimator.update();
//     }

//     public void update(Pose2d robotPose) {
//         visionSim.update(robotPose);
//     }

//     public void updateCameraRotation(Rotation3d pose) {
//         Transform3d robotToCamera = new Transform3d(
//             robotToCameraTrl.rotateBy(pose),
//             robotToCameraRot.rotateBy(pose));
//         visionSim.adjustCamera(cameraSim, robotToCamera);
//     }

// }
