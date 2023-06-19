package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;
import com.spartronics4915.lib.T265Camera;

import org.jetbrains.annotations.NotNull;

import java.util.Locale;
import java.util.function.Consumer;

/**
 * This class implements our VSLAM camera. It holds on to an instance of the camera in a
 * static class member. It also maintains other variables like the current position,
 * velocity etc.
 * <p>
 * One should call start and stop on this class to start and stop the camera. The constructor
 * starts the camera.
 * <p>
 * This class implements the localizer interface of RoadRunner to be able to be used in that
 * library.
 * <p>
 * It also implements the T265Camera.CameraUpdate interface to accept position updates from the
 * camera in an asynchronous manner.
 */

public class VslamLocalizer implements Localizer, Consumer<T265Camera.CameraUpdate> {
    //Remember all measurements for ftc-lib geometry are in meters

    //How far back the center of the robot is from the camera
    public static final double CAMERA_OFFSET_FRONT =
            -DriveConstants.VSLAM_CAMERA_FORWARD_OFFSET_MM / 1000; //in meters
    //How far left of the camera the center is
    public static final double CENTER_OFFSET_Y = DriveConstants.VSLAM_CAMERA_LEFT_OFFSET_MM / 1000;
    public static final double T265_ROTATION = DriveConstants.VSLAM_CAMERA_ROTATION;
    public static final Transform2d robotCenterOffsetFromCamera =
            new Transform2d(new Translation2d(CAMERA_OFFSET_FRONT, CENTER_OFFSET_Y),
                    new Rotation2d(Math.toRadians(T265_ROTATION)));

    //we maintain a static camera instance

    private static double M_PER_INCH =  0.0254;
    private static T265Camera t265Camera = null;
    //and a static synchronization object
    private static final Object synchronizationObject = new Object();

    private Pose2d currentPose = new Pose2d();
    private com.arcrobotics.ftclib.geometry.Pose2d initialPose = new com.arcrobotics.ftclib.geometry.Pose2d();
    private T265Camera.CameraUpdate lastCameraUpdate;
    private Pose2d pose2dVelocity = new Pose2d();

    public MecanumDrive.MecanumLocalizer mecanumLocalizer = null;
    private volatile boolean isInitialized;

    private com.arcrobotics.ftclib.geometry.Transform2d originOffset =
            new com.arcrobotics.ftclib.geometry.Transform2d();

    /**
     * Constructor. Note that you call this constructor with the hardwareMap to get an instance
     * of the vslam camera. The constructor creates a thread to create the instance and then to
     * start it.
     *
     * @param hardwareMap - hardware map
     */
    public VslamLocalizer(HardwareMap hardwareMap, MecanumDrive.MecanumLocalizer mecanumLocalizer) {
        this.mecanumLocalizer = mecanumLocalizer;
        isInitialized = false;
        Thread cameraInitializationThread = new Thread(() -> {
            synchronized (synchronizationObject) {
                if (t265Camera == null) {
                    t265Camera = new T265Camera(robotCenterOffsetFromCamera, 0.8, hardwareMap.appContext);
                } else {
                }
                start();
                isInitialized = true;
            }
        });
        cameraInitializationThread.start();
    }

    /**
     * Set the current position of the robot. We simply remember the offset from what the camera is telling us
     *
     * @param pose the pose to set
     */
    public synchronized void setCurrentPose(Pose2d pose) {
        setCurrentPose(roadRunnerToCameraPose(pose));
    }

    /**
     * Set the current position of the robot. We simply remember the offset from what the camera is telling us
     *
     * @param newPose the pose to set
     */
    public synchronized void setCurrentPose(com.arcrobotics.ftclib.geometry.Pose2d newPose) {
        synchronized (synchronizationObject) {
            initialPose = newPose;
            if (lastCameraUpdate != null){
            originOffset =
                    newPose.minus(lastCameraUpdate.pose);
            //force an update to current pose
            accept(lastCameraUpdate);

            t265Camera.setPose(newPose);}
        }
    }

    public synchronized boolean isInitialized() {
        synchronized (synchronizationObject) {
            return isInitialized;
        }
    }

    //Methods to implement roadrunner localizer

    /**
     * Returns the current pose. The pose units of measure are inches in x and y coordinates and
     * radians for the heading.
     */
    @NotNull
    @Override
    public Pose2d getPoseEstimate() {
        synchronized (synchronizationObject) {
            return currentPose;
        }
    }

    /**
     * Implementation of the update method required by the Localizer interface.
     * We don't do anything here as the updates are handled in an asynchronous manner by implementing
     * the accept method of the T265Camera interface
     */
    @Override
    public void update() {
        //mecanumLocalizer.update();
        //t265Camera.sendOdometry(mecanumLocalizer.getPoseVelocity().getX() * M_PER_INCH, mecanumLocalizer.getPoseVelocity().getY() * M_PER_INCH);
    }

    /**
     *
     * Method to set current pose estimate, as required by the RoadRunner interface. Since we
     * are using a VSLAM camera, we don't do anything here as we are totally dependent on the
     * camera provided positions.
     */
    @Override
    public void setPoseEstimate(@NotNull Pose2d pose2d) {
        setCurrentPose(pose2d);
    }

    @Override
    public Pose2d getPoseVelocity() {
        synchronized (synchronizationObject) {
            return pose2dVelocity;
        }
    }

    /**
     * Method to start the VSLAM camera. We catch the "Camera is already started" exception just in
     * case this method gets called while the camera was already running.
     */
    public void start() {
        synchronized (synchronizationObject) {
            lastCameraUpdate = null;
            try {//start our camera only if it is not already initialized
                if (!isInitialized) {
                    //if the static camera was already started, stop it first
                    if (t265Camera.isStarted()) {
                        t265Camera.stop();
                    }
                    //start the camera so we can start receiving updates
                    t265Camera.start(this);
                }
            } catch (Throwable e) {
                RobotLog.logStackTrace("SilverTitans: Error starting real sense", e);
            }
        }
    }

    /**
     * Accept the asynchronous update sent by the camera.
     * Our current pose is kept in inches and radians. We also maintain our pose velocity
     * by converting the velocity provided to inches/second.
     *
     * @param cameraUpdate - the camera update provided by the camera
     */
    @Override
    public void accept(T265Camera.CameraUpdate cameraUpdate) {
        synchronized (synchronizationObject) {
            //process latest received update
            try {
                pose2dVelocity = new Pose2d(
                        cameraUpdate.velocity.vxMetersPerSecond / M_PER_INCH,
                        cameraUpdate.velocity.vyMetersPerSecond / M_PER_INCH,
                        cameraUpdate.velocity.omegaRadiansPerSecond);
                lastCameraUpdate = cameraUpdate;
                //set last pose to be the current one
                com.arcrobotics.ftclib.geometry.Pose2d newPose = cameraUpdate.pose.plus(
                        originOffset);
                currentPose = cameraToRoadRunnerPose(cameraUpdate.pose);
            } catch (Throwable e) {
                RobotLog.ee("Vslam", e, "Error getting position");
            }
        }
    }

    public String getStatus() {
        synchronized (synchronizationObject) {
            if (havePosition()) {
                return String.format(Locale.getDefault(), "%s,Camera:%s,Initial:%s,Confidence:%s",
                        getPoseEstimate(),
                        cameraToRoadRunnerPose(lastCameraUpdate.pose),
                        cameraToRoadRunnerPose(initialPose),
                        getPoseConfidence().toString());
            } else {
                return "No camera update yet";
            }
        }
    }


    public boolean havePosition() {
        synchronized (synchronizationObject) {
            return lastCameraUpdate != null;
        }
    }

    public T265Camera.PoseConfidence getPoseConfidence() {
        synchronized (synchronizationObject) {
            return lastCameraUpdate.confidence;
        }
    }

    private static com.arcrobotics.ftclib.geometry.Pose2d roadRunnerToCameraPose(Pose2d roadRunnerPose) {
        return new com.arcrobotics.ftclib.geometry.Pose2d
                (roadRunnerPose.getX() * M_PER_INCH, roadRunnerPose.getY() * M_PER_INCH,
                        new Rotation2d(roadRunnerPose.getHeading()));
    }
    private static Pose2d cameraToRoadRunnerPose(com.arcrobotics.ftclib.geometry.Pose2d cameraPose) {
        return new Pose2d(
                cameraPose.getTranslation().getX() / M_PER_INCH,
                cameraPose.getTranslation().getY() / M_PER_INCH,
                cameraPose.getHeading());
    }
    public static void main(String[] args) {
        com.arcrobotics.ftclib.geometry.Pose2d setPose =
                new com.arcrobotics.ftclib.geometry.Pose2d(
                        -0.78,
                        -1.59,
                        new Rotation2d(Math.toRadians(-90)));
        System.out.println("Setting initial pose to be: " + setPose);

        com.arcrobotics.ftclib.geometry.Pose2d initialCameraPose =
                roadRunnerToCameraPose(new Pose2d(
                        -0.02,
                        1.19,
                        Math.toRadians(-2.36)));
        System.out.println("Initial camera pose = " + initialCameraPose);

        com.arcrobotics.ftclib.geometry.Transform2d offset = setPose.minus(initialCameraPose);
        //System.out.println("Offset=" + Field.cameraToRoadRunnerPose(offset));

        com.arcrobotics.ftclib.geometry.Pose2d newCameraPose = roadRunnerToCameraPose(
               new Pose2d(-20.02, 21.19, Math.toRadians(-2.36)));
        System.out.println("New camera pose = " + newCameraPose);

        com.arcrobotics.ftclib.geometry.Pose2d newLocation =
                newCameraPose.plus(offset);
        //offset.transformBy(new Transform2d(newCameraPose.getTranslation(), newCameraPose.getRotation()));
        System.out.println("Result = " + newLocation);

        System.out.println("Resultant camera movement = " + cameraToRoadRunnerPose(
                newCameraPose.relativeTo(initialCameraPose)));
        System.out.println("Resultant robot movement = " + cameraToRoadRunnerPose(
                newLocation.relativeTo(setPose)));
    }


}

