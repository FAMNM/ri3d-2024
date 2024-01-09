package frc.robot.subsystems;

import java.util.ArrayList;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {

    public Vision() {
        var visionThread = new Thread(this::apriltagVisionThreadProc);
        visionThread.setDaemon(true);
        visionThread.start();
    }

    void apriltagVisionThreadProc() {
        var detector = new AprilTagDetector();
        // look for tag136h11, correct 7 error bits
        detector.addFamily("tag36h11", 7);

        var poseEstConfig = new AprilTagPoseEstimator.Config(
            Constants.VisionConstants.APRILTAG_SIZE,
            Constants.VisionConstants.CAMERA_CX,
            Constants.VisionConstants.CAMERA_CY,
            Constants.VisionConstants.CAMERA_FX,
            Constants.VisionConstants.CAMERA_FY);
        var estimator = new AprilTagPoseEstimator(poseEstConfig);

        // Get the UsbCamera from CameraServer
        UsbCamera camera = CameraServer.startAutomaticCapture();
        // Set the resolution
        camera.setResolution(640, 480);

        // Get a CvSink. This will capture Mats from the camera
        CvSink cvSink = CameraServer.getVideo();
        // Setup a CvSource. This will send images back to the Dashboard
        CvSource outputStream = CameraServer.putVideo("detect", 640, 480);

        // Mats are very memory expensive. Lets reuse these.
        var mat = new Mat();
        var grayMat = new Mat();

        // Instantiate once
        ArrayList<Long> tags = new ArrayList<>();
        var outlineColor = new Scalar(0, 255, 0);
        var crossColor = new Scalar(0, 0, 255);

        // We'll output to NT
        NetworkTable tagsTable = NetworkTableInstance.getDefault().getTable("apriltags");
        IntegerArrayPublisher pubTags = tagsTable.getIntegerArrayTopic("tags").publish();

        // This cannot be 'true'. The program will never exit if it is. This
        // lets the robot stop this thread when restarting robot code or
        // deploying.
        while (!Thread.interrupted()) {
            // Tell the CvSink to grab a frame from the camera and put it
            // in the source mat.  If there is an error notify the output.
            if (cvSink.grabFrame(mat) == 0) {
                // Send the output the error.
                outputStream.notifyError(cvSink.getError());
                // skip the rest of the current iteration
                continue;
            }

            Imgproc.cvtColor(mat, grayMat, Imgproc.COLOR_RGB2GRAY);

            AprilTagDetection[] detections = detector.detect(grayMat);

            // have not seen any tags yet
            tags.clear();

            for (AprilTagDetection detection : detections) {
                // remember we saw this tag
                tags.add((long) detection.getId());

                // draw lines around the tag
                for (var i = 0; i <= 3; i++) {
                    var j = (i + 1) % 4;
                    var pt1 = new Point(detection.getCornerX(i), detection.getCornerY(i));
                    var pt2 = new Point(detection.getCornerX(j), detection.getCornerY(j));
                    Imgproc.line(mat, pt1, pt2, outlineColor, 2);
                }

                // mark the center of the tag
                var cx = detection.getCenterX();
                var cy = detection.getCenterY();
                var ll = 10;
                Imgproc.line(mat, new Point(cx - ll, cy), new Point(cx + ll, cy), crossColor, 2);
                Imgproc.line(mat, new Point(cx, cy - ll), new Point(cx, cy + ll), crossColor, 2);

                // identify the tag
                Imgproc.putText(
                    mat,
                    Integer.toString(detection.getId()),
                    new Point(cx + ll, cy),
                    Imgproc.FONT_HERSHEY_SIMPLEX,
                    1,
                    crossColor,
                    3);

                // determine pose
                Transform3d pose = estimator.estimate(detection);

                // put pose into dashboard
                Rotation3d rot = pose.getRotation();
                tagsTable
                    .getEntry("pose_" + detection.getId())
                    .setDoubleArray(
                        new double[] {
                        pose.getX(), pose.getY(), pose.getZ(), rot.getX(), rot.getY(), rot.getZ()
                        });
            }

            // put list of tags onto dashboard
            pubTags.set(tags.stream().mapToLong(Long::longValue).toArray());

            // Give the output stream a new image to display
            outputStream.putFrame(mat);
        }

        pubTags.close();
        detector.close();
    }

    // private final UsbCamera camera;
    // private final CvSink cvSink;
    // private Mat mat; // image
    // private Mat grayMat; // image
    // private final AprilTagDetector detector;
    // private final AprilTagPoseEstimator poseEstimator;
    // private final Map<Integer, Transform3d> poseEstimations;
    // private final Optional<AprilTagFieldLayout> field;
    // private final Thread visionThread;
    // private final CvSource outputStream;

    // public Vision() {
    //     super("Vision");
    //     // Start & Config Camera
    //     camera = CameraServer.startAutomaticCapture();
    //     camera.setResolution(Constants.VisionConstants.RESOLUTION_WIDTH, Constants.VisionConstants.RESOLUTION_WIDTH);
    //     cvSink = CameraServer.getVideo();
    //     outputStream = CameraServer.putVideo("detect", 640, 480);
    //     mat = new Mat();
    //     grayMat = new Mat();

    //     // Configure April Tag Field
    //     field = getField();

    //     // April Tag Detector Initialization
    //     detector = new AprilTagDetector();
    //     detector.addFamily(Constants.VisionConstants.FAMILY, 7);

    //     // April Tag Pose Estimator Initialization
    //     AprilTagPoseEstimator.Config estimatorConfig = new AprilTagPoseEstimator.Config(
    //             Constants.VisionConstants.APRILTAG_SIZE,
    //             Constants.VisionConstants.CAMERA_CX,
    //             Constants.VisionConstants.CAMERA_CY,
    //             Constants.VisionConstants.CAMERA_FX,
    //             Constants.VisionConstants.CAMERA_FY);
    //     poseEstimator = new AprilTagPoseEstimator(estimatorConfig);
    //     poseEstimations = new TreeMap<Integer, Transform3d>();
    //     visionThread = new Thread(() -> {
    //         while (!Thread.interrupted()) {
    //             process();
    //         }
    //     });
    //     visionThread.start();
    // }

    // public void process() {
    //     // Read Frame from Camera
    //     if (cvSink.grabFrame(mat) == 0) {
    //         outputStream.notifyError(cvSink.getError());
    //         return;
    //     }


    //     // WPILib April Tag Recognition & Pose Estimation
    //     Imgproc.cvtColor(mat, grayMat, Imgproc.COLOR_RGB2GRAY);
    //     AprilTagDetection[] detections = detector.detect(grayMat);
    //     poseEstimations.clear();

    //     Scalar outlineColor = new Scalar(0, 255, 0);
    //     Scalar xColor = new Scalar(0, 0, 255);
    //     for (AprilTagDetection detection : detections) {
    //         for (var i = 0; i <= 3; i++) {
    //             var j = (i + 1) % 4;
    //             var pt1 = new Point(detection.getCornerX(i), detection.getCornerY(i));
    //             var pt2 = new Point(detection.getCornerX(j), detection.getCornerY(j));
    //             Imgproc.line(grayMat, pt1, pt2, outlineColor, 2);
    //         }

    //         var cx = detection.getCenterX();
    //         var cy = detection.getCenterY();
    //         var ll = 10;
    //         Imgproc.line(grayMat, new Point(cx - ll, cy), new Point(cx + ll, cy), xColor, 2);
    //         Imgproc.line(grayMat, new Point(cx, cy - ll), new Point(cx, cy + ll), xColor, 2);
    //         Imgproc.putText(grayMat, Integer.toString(detection.getId()), new Point (cx + ll, cy), Imgproc.FONT_HERSHEY_SIMPLEX, 1, xColor, 3);
    //     }
    //     outputStream.putFrame(grayMat);

    //     String[] detectionStrings = new String[detections.length];
    //     for (int i = 0; i < detections.length; i++) {
    //         detectionStrings[i] = detections[i].toString();
    //     }
    //     SmartDashboard.putStringArray("Detections", detectionStrings);
    //     for (var detection : detections) {
    //         System.out.println("DETECTION!");
    //         poseEstimations.put(detection.getId(), poseEstimator.estimate(detection));
    //     }

    //     for (var estimate : poseEstimations.entrySet()) {
    //         SmartDashboard.putString("April tag #" + estimate.getKey(), estimate.getValue().toString());
    //     }

    //     // if (field.isPresent()) {
    //     //     analyze();
    //     // }

    //     // // Display robot offset to grid on smartdashboard
    //     // if (DriverStation.getAlliance().isEmpty()) {
    //     //     return;
    //     // }
    //     // if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
    //     //     if (poseEstimations.containsKey(1)) {
    //     //         SmartDashboard.putNumber("grid_offset", poseEstimations.get(1).getY());
    //     //     } else if (poseEstimations.containsKey(2)) {
    //     //         SmartDashboard.putNumber("grid_offset", poseEstimations.get(2).getY());
    //     //     } else if (poseEstimations.containsKey(3)) {
    //     //         SmartDashboard.putNumber("grid_offset", poseEstimations.get(2).getY());
    //     //     }
    //     // } else {
    //     //     if (poseEstimations.containsKey(6)) {
    //     //         SmartDashboard.putNumber("grid_offset", poseEstimations.get(6).getY());
    //     //     } else if (poseEstimations.containsKey(7)) {
    //     //         SmartDashboard.putNumber("grid_offset", poseEstimations.get(7).getY());
    //     //     } else if (poseEstimations.containsKey(8)) {
    //     //         SmartDashboard.putNumber("grid_offset", poseEstimations.get(8).getY());
    //     //     }
    //     // }
    // }

    // private Optional<AprilTagFieldLayout> getField() {
    //     try {
    //         return Optional.of(AprilTagFieldLayout
    //                 .loadFromResource((new File(Filesystem.getDeployDirectory(), "2024-crescendo.json")).toString()));
    //     } catch (IOException ioe) {
    //         System.err.println("Failed to open April Tag Field Location Configuration File");
    //         ioe.printStackTrace();
    //         return Optional.empty();
    //     }
    // }

    // private void analyze() {
    //     // Convert Robot-Relative April Tg Pose to Robot Pos on Field
    //     Translation2d avgRobotPos = new Translation2d();
    //     Rotation2d avgRobotRot = new Rotation2d();

    //     for (var estimate : poseEstimations.entrySet()) {
    //         // Transform locations of april tags to get robot pos
    //         Pose3d tagPose = field.get().getTagPose(estimate.getKey()).get();
    //         Transform3d tagToRobot = Constants.VisionConstants.ROBOT_CENTER_TO_CAMERA.plus(estimate.getValue());
    //         Pose3d robot3d = tagPose.transformBy(tagToRobot.inverse());
    //         Pose2d robot2d = robot3d.toPose2d(); // At this point Z should be 0

    //         // Sum pos components
    //         avgRobotPos.plus(robot2d.getTranslation());
    //         avgRobotRot.plus(robot2d.getRotation());
    //     }

    //     // Take average of cumulative poses
    //     avgRobotPos.times(1d / ((double) poseEstimations.size()));
    //     avgRobotRot.times(1d / ((double) poseEstimations.size()));
    //     Pose2d robotPose = new Pose2d(avgRobotPos, avgRobotRot);
    //     SmartDashboard.putString("robot_pose", robotPose.toString());
    // }
}
