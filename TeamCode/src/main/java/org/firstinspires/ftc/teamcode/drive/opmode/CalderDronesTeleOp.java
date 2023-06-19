package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Lift;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Config
@TeleOp(group = "drive")
public class CalderDronesTeleOp extends LinearOpMode {
    private Lift lift;

    private Gamepad currentGamepad1 = new Gamepad();
    private Gamepad currentGamepad2 = new Gamepad();
    private Gamepad previousGamepad1 = new Gamepad();
    private Gamepad previousGamepad2 = new Gamepad();
    public double power_multiplier = 1.0;
    @Override
    public void runOpMode() throws InterruptedException {
        lift = new Lift(hardwareMap, this);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        lift.teleOpInitialise();

        waitForStart();

        ElapsedTime timer = new ElapsedTime();

        while (!isStopRequested()) {

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            lift.update(currentGamepad2, previousGamepad2);

            if(currentGamepad1.left_bumper) {
                power_multiplier = 0.25;
            }
            else if (currentGamepad1.right_bumper){
                power_multiplier = 1.0;
            }
            else {
                power_multiplier = 0.4;
            }

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * power_multiplier,
                            -gamepad1.left_stick_x * power_multiplier,
                            -gamepad1.right_stick_x * power_multiplier
                    )
            );

            drive.update();
            // Show the elapsed game time and wheel power.
            telemetry.addData("lift_left Position", "lift_left (%d)", lift.lift_left.getCurrentPosition());
            telemetry.addData("lift_right Position", "lift_left (%d)", lift.lift_left.getCurrentPosition());
            telemetry.addData("lift_set_point", "lift_set_point (%d)", lift.lift_set_point);
            telemetry.addData("lift set point name", "lift_set_point_name (%s)", lift.liftTargetPreset);
            telemetry.addData("gripper", "gripper (%f)", lift.gripper.getPosition());
            telemetry.addData("lift_power", "lift_power (%f)", lift.out);
            telemetry.update();


        }
    }
}
