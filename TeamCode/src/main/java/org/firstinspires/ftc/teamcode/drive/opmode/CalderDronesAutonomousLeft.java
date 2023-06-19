package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.teamcode.drive.Lift;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;

/*
 * Op mode for preliminary tuning of the follower PID coefficients (located in the drive base
 * classes). The robot drives in a DISTANCE-by-DISTANCE square indefinitely. Utilization of the
 * dashboard is recommended for this tuning routine. To access the dashboard, connect your computer
 * to the RC's WiFi network. In your browser, navigate to https://192.168.49.1:8080/dash if you're
 * using the RC phone or https://192.168.43.1:8080/dash if you are using the Control Hub. Once
 * you've successfully connected, start the program, and your robot will begin driving in a square.
 * You should observe the target position (green) and your pose estimate (blue) and adjust your
 * follower PID coefficients such that you follow the target position as accurately as possible.
 * If you are using SampleMecanumDrive, you should be tuning TRANSLATIONAL_PID and HEADING_PID.
 * If you are using SampleTankDrive, you should be tuning AXIAL_PID, CROSS_TRACK_PID, and HEADING_PID.
 * These coefficients can be tuned live in dashboard.
 */
@Config
@Autonomous(group = "drive")
public class CalderDronesAutonomousLeft extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.05;
    AprilTagDetection tagOfInterest = null;
    public static double DISTANCE = 48; // in
    private Lift lift;

    private Pose2d startingPose;

    private Pose2d waypoint1Pose;
    private double waypoint1Heading;
    private Pose2d waypoint2Pose;
    private double waypoint2Heading;
    private Pose2d waypoint3Pose;
    private double waypoint3Heading;

    private Pose2d highPolePose;
    private double highPoleHeading;

    private Pose2d parkingWaypointPose;
    private double parkingWaypointHeading;

    private Pose2d parkingLocation;
    private double parkingHeading;

    private int parkingZone = 3;
    String alliance = "Red";
    String side = "Left";
    String startingState = alliance + side;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);
        lift = new Lift(hardwareMap, this);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.autonomousInitialise();
        //lift.choosePresetPosition("GroundStation");
        //while (!lift.liftReady()){
        //    lift.update();
        //}
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections) {

                    tagOfInterest = tag;
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                    tagFound = true;
                }
            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

            }

            telemetry.update();
            sleep(20);
        }

        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
            parkingZone = tagOfInterest.id;
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }


        switch (startingState) {
            case "BlueLeft":
                startingPose = new Pose2d(36, 62, Math.toRadians(270));
                waypoint1Pose = new Pose2d(36, -14, Math.toRadians(90));
                waypoint1Heading = Math.toRadians(90);
                waypoint2Pose = new Pose2d(12, -48, Math.toRadians(90));
                waypoint2Heading = Math.toRadians(90);
                waypoint3Pose = new Pose2d(14, -24, Math.toRadians(90));
                waypoint3Heading = Math.toRadians(90);
                highPolePose = new Pose2d(29.5, 6, Math.toRadians(225));
                highPoleHeading = Math.toRadians(245);

                switch (parkingZone) {
                    case 1:
                        parkingWaypointPose = new Pose2d(-11.75, 11.75, Math.toRadians(270));;
                        parkingWaypointHeading = Math.toRadians(0);;
                        parkingLocation = new Pose2d(11.75, 11.75, Math.toRadians(270));
                        parkingHeading = Math.toRadians(180);
                        break;
                    case 2:
                        parkingWaypointPose = new Pose2d(-11.75, 11.75, Math.toRadians(270));;
                        parkingWaypointHeading = Math.toRadians(0);;
                        parkingLocation = new Pose2d(35.25, 23.5, Math.toRadians(270));
                        parkingHeading = Math.toRadians(90);
                        break;
                    case 3:
                    default:
                        parkingWaypointPose = new Pose2d(-11.75, 11.75, Math.toRadians(270));;
                        parkingWaypointHeading = Math.toRadians(0);;
                        parkingLocation = new Pose2d(58.75, 11.75, Math.toRadians(180));
                        parkingHeading = Math.toRadians(0);
                        break;
                }
                break;
            case "BlueRight":
                startingPose = new Pose2d(-36, 62, Math.toRadians(270));
                waypoint1Pose = new Pose2d(36, -14, Math.toRadians(90));
                waypoint1Heading = Math.toRadians(90);
                waypoint2Pose = new Pose2d(12, -48, Math.toRadians(90));
                waypoint2Heading = Math.toRadians(90);
                waypoint3Pose = new Pose2d(14, -24, Math.toRadians(90));
                waypoint3Heading = Math.toRadians(90);
                highPolePose = new Pose2d(-29.5, 6, Math.toRadians(315));
                highPoleHeading = Math.toRadians(295);

                switch (parkingZone) {
                    case 1:
                        parkingWaypointPose = new Pose2d(-11.75, 11.75, Math.toRadians(270));;
                        parkingWaypointHeading = Math.toRadians(0);;
                        parkingLocation = new Pose2d(-11.75, 11.75, Math.toRadians(270));
                        parkingHeading = Math.toRadians(0);
                        break;
                    case 2:
                        parkingWaypointPose = new Pose2d(-11.75, 11.75, Math.toRadians(270));;
                        parkingWaypointHeading = Math.toRadians(0);;
                        parkingLocation = new Pose2d(-35.25, 23.5, Math.toRadians(270));
                        parkingHeading = Math.toRadians(90);
                        break;
                    case 3:
                    default:
                        parkingWaypointPose = new Pose2d(-11.75, 11.75, Math.toRadians(270));;
                        parkingWaypointHeading = Math.toRadians(0);;
                        parkingLocation = new Pose2d(-58.75, 11.75, Math.toRadians(270));
                        parkingHeading = Math.toRadians(180);
                        break;
                }
                break;
            case "RedLeft":
                startingPose = new Pose2d(-36, -62, Math.toRadians(90));
                waypoint1Pose = new Pose2d(-24, -58, Math.toRadians(0));
                waypoint1Heading = Math.toRadians(0);
                waypoint2Pose = new Pose2d(-14, -48, Math.toRadians(90));
                waypoint2Heading = Math.toRadians(90);
                waypoint3Pose = new Pose2d(-14, -24, Math.toRadians(90));
                waypoint3Heading = Math.toRadians(90);
                highPolePose = new Pose2d(-23, -4, Math.toRadians(90));
                highPoleHeading = Math.toRadians(90);

                switch (parkingZone) {
                    case 1:
                        parkingWaypointPose = new Pose2d(-48, -14, Math.toRadians(0));;
                        parkingWaypointHeading = Math.toRadians(180);;
                        parkingLocation = new Pose2d(-60, -24, Math.toRadians(90));
                        parkingHeading = Math.toRadians(270);
                        break;
                    case 2:
                        parkingWaypointPose = new Pose2d(-32, -18, Math.toRadians(45));;
                        parkingWaypointHeading = Math.toRadians(225);;
                        parkingLocation = new Pose2d(-36, -24, Math.toRadians(90));
                        parkingHeading = Math.toRadians(270);
                        break;
                    case 3:
                    default:
                        parkingWaypointPose = new Pose2d(-14, -18, Math.toRadians(135));;
                        parkingWaypointHeading = Math.toRadians(315);;
                        parkingLocation = new Pose2d(-12, -24, Math.toRadians(90));
                        parkingHeading = Math.toRadians(270);
                        break;
                }
                break;
            case "RedRight":
            default:
                startingPose = new Pose2d(36, -62, Math.toRadians(90));
                waypoint1Pose = new Pose2d(24, -58, Math.toRadians(180));
                waypoint1Heading = Math.toRadians(180);
                waypoint2Pose = new Pose2d(14, -48, Math.toRadians(90));
                waypoint2Heading = Math.toRadians(90);
                waypoint3Pose = new Pose2d(14, -24, Math.toRadians(90));
                waypoint3Heading = Math.toRadians(90);
                highPolePose = new Pose2d(23, -4, Math.toRadians(90));
                highPoleHeading = Math.toRadians(90);

                switch (parkingZone) {
                    case 1:
                        parkingWaypointPose = new Pose2d(14, -18, Math.toRadians(45));;
                        parkingWaypointHeading = Math.toRadians(225);;
                        parkingLocation = new Pose2d(12, -24, Math.toRadians(90));
                        parkingHeading = Math.toRadians(270);
                        break;
                    case 2:
                        parkingWaypointPose = new Pose2d(32, -18, Math.toRadians(135));;
                        parkingWaypointHeading = Math.toRadians(315);;
                        parkingLocation = new Pose2d(36, -24, Math.toRadians(90));
                        parkingHeading = Math.toRadians(270);
                        break;
                    case 3:
                    default:
                        parkingWaypointPose = new Pose2d(48, -14, Math.toRadians(180));;
                        parkingWaypointHeading = Math.toRadians(0);;
                        parkingLocation = new Pose2d(60, -24, Math.toRadians(90));
                        parkingHeading = Math.toRadians(270);
                        break;
                }
                break;
        }
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startingPose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    lift.choosePresetPosition("GroundStation");
                    telemetry.addData("Status:", "Moving lift to ground position");
                    telemetry.update();
                })

                .splineToSplineHeading(waypoint1Pose, waypoint1Heading)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    lift.choosePresetPosition("HighPole");
                    telemetry.addData("Status:", "Moving lift to High position");
                    telemetry.update();
                })
                .splineToSplineHeading(waypoint2Pose, waypoint2Heading)
                .splineToSplineHeading(waypoint3Pose, waypoint3Heading)
                .splineToSplineHeading(highPolePose, highPoleHeading)

                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                    lift.choosePresetPosition("HighPoleDrop");
                    telemetry.addData("Status:", "Moving lift to HighDrop position");
                    telemetry.update();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> {
                    lift.gripper.dropCone();
                    telemetry.addData("Status:", "Dropping Cone");
                    telemetry.update();
                })

                .waitSeconds(1.5) //wait for cone to drop

                .UNSTABLE_addTemporalMarkerOffset(0.5,() -> {
                    lift.choosePresetPosition("GroundStation");
                })

                //drive to parking area
                .setReversed(true)
                .splineToSplineHeading(parkingWaypointPose, parkingWaypointHeading)
                .splineToSplineHeading(parkingLocation, parkingHeading)

                .build();



        drive.setPoseEstimate(startingPose);
        drive.followTrajectorySequenceAsync(trajSeq);
        while (!isStopRequested()) {
            drive.update();
            lift.autoUpdate();

        }
    }
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}
