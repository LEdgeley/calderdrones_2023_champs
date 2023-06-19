package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;

@Config
public class Lift {
    public DcMotor lift_left ;
    public DcMotor lift_right ;
    public Gripper gripper;
    public Positioner positioner;
    public ElapsedTime stateMachineTimer;
    private ElapsedTime PIDTimer;
    private LinearOpMode opMode;
    public enum LiftState{
        CONE_POSITIONING,
        LOWERING_LIFT_TO_GRAB,
        GRABBING_CONE,
        POSITIONER_RELEASING,
        LIFT_RAISING,
        LOWERING_LIFT_TO_DROP,
        CONE_DROPPING,
        READY
    }

    private LiftState liftState;

    public int lift_set_point = 0;
    private double integral_sum = 0;
    private double lastError = 0;

    public double lift_power = -0.1;
    public static double Kp = 0.01;
    public static double Ki = 0.0;
    public static double Kd = 0.0;
    public static double Kg = 0.27;

    public double out = 0.0;
    public LinkedHashMap<String, Integer> presets = new LinkedHashMap<String, Integer>();
    public String liftTargetPreset = "Floor";

    public Lift(HardwareMap hardwareMap, LinearOpMode opMode){
        this.opMode = opMode;
        gripper = new Gripper(hardwareMap);
        positioner = new Positioner(hardwareMap);
        stateMachineTimer = new ElapsedTime();
        PIDTimer = new ElapsedTime();

        presets.put("Floor", 0);
        presets.put("OneCone", 62);
        presets.put("GroundStationDrop", 240);
        presets.put("GroundStation", 240);
        presets.put("2StackedCones", 240);
        presets.put("3StackedCones", 260);
        presets.put("4StackedCones", 280);
        presets.put("5StackedCones", 300);
        presets.put("LowPoleDrop", 375);
        presets.put("LowPole", 425);
        presets.put("MedPoleDrop", 500);
        presets.put("MedPole", 580);
        presets.put("HighPoleDrop", 670);
        presets.put("HighPole", 760);

        lift_left  = hardwareMap.get(DcMotor.class, "lift_left");
        lift_right  = hardwareMap.get(DcMotor.class, "lift_right");

        lift_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //This assumes the robot starts with the lift retracted (at minExtenstion)
        lift_left.setTargetPosition(presets.get("Floor"));
        lift_right.setTargetPosition(presets.get("Floor"));

        lift_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lift_left.setDirection(DcMotor.Direction.REVERSE);
        lift_right.setDirection(DcMotor.Direction.REVERSE);
    }
    public void lowerLift() throws InterruptedException {
        lift_left.setPower(-0.1);
        lift_right.setPower(-0.1);
    }

    public void forceLowerLift() throws InterruptedException {
        lift_left.setPower(-0.5);
        lift_right.setPower(-0.5);
    }

    public int getPosition(){
        return lift_set_point;
    }

    public void setPosition(int position){
        lift_set_point = position;
    }
    public void changePosition(int offset){
        lift_set_point += offset;
    }
    public void choosePresetPosition(String preset) {
        liftTargetPreset = preset;

        lift_set_point = presets.get(liftTargetPreset);
    }

    public void nextPolePosition() {
        if(liftTargetPreset == "GroundStation") {
            liftTargetPreset = "LowPole";
        }
        else if (liftTargetPreset == "LowPole"){
            liftTargetPreset = "MedPole";
        }
        else if(liftTargetPreset == "MedPole") {
            liftTargetPreset = "HighPole";
        }
        else {
            nextPresetPosition();
        }
        lift_set_point = presets.get(liftTargetPreset);
    }

    public void previousPolePosition() {
        if(liftTargetPreset == "LowPole") {
            liftTargetPreset = "GroundStation";
        }
        else if (liftTargetPreset == "MedPole"){
            liftTargetPreset = "LowPole";
        }
        else if(liftTargetPreset == "HighPole") {
            liftTargetPreset = "MedPole";
        }
        else{
            previousPresetPosition();
        }
        lift_set_point = presets.get(liftTargetPreset);
    }
    public void nextPresetPosition() {
        List<String> keys = new ArrayList<>(presets.keySet());
        for (int k = 0; k < keys.size(); k ++) {
            if (keys.get(k) == liftTargetPreset && k < keys.size() -1) {
                liftTargetPreset = keys.get(k + 1);
                lift_set_point = presets.get(liftTargetPreset);
                return;
            }
        }
    }

    public void previousPresetPosition() {
        List<String> keys = new ArrayList<>(presets.keySet());
        for (int k = 0; k < keys.size(); k ++) {
            if (keys.get(k) == liftTargetPreset && k > 0) {
                liftTargetPreset = keys.get(k - 1);
                lift_set_point = presets.get(liftTargetPreset);
                return;
            }
        }
    }
    public void teleOpInitialise() throws InterruptedException {

        positioner.releaseCone();
        gripper.dropCone();
        opMode.telemetry.addData("Status", "Lowering Lift to Stop");
        opMode.telemetry.update();
        lowerLift();
        opMode.sleep(500);
        forceLowerLift();
        opMode.sleep(500);

        lift_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift_left.setPower(0.0);
        lift_right.setPower(0.0);

        lift_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        opMode.telemetry.addData("Status", "Initalised");
        opMode.telemetry.update();

        liftState = LiftState.READY;
        stateMachineTimer.reset();
        PIDTimer.reset();

    }

    public void autonomousInitialise() throws InterruptedException {

        opMode.telemetry.addData("Status", "Positioning Cone");
        opMode.telemetry.update();
        positioner.positionCone();
        opMode.sleep(250);

        opMode.telemetry.addData("Status", "Lowering Lift to Stop");
        opMode.telemetry.update();
        lowerLift();
        opMode.sleep(1000);
        forceLowerLift();
        opMode.sleep(500);

        lift_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift_left.setPower(0.0);
        lift_right.setPower(0.0);

        lift_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        opMode.telemetry.addData("Status", "Grabbing Cone");
        opMode.telemetry.update();
        gripper.grabCone();
        opMode.sleep(250);

        opMode.telemetry.addData("Status", "Releasing Positioner");
        opMode.telemetry.update();
        positioner.releaseCone();
        opMode.sleep(250);

        opMode.telemetry.addData("Status", "Initalised");
        opMode.telemetry.update();

        liftState = LiftState.READY;
        stateMachineTimer.reset();
        PIDTimer.reset();

    }
    public void setLiftState(LiftState newState)
    {
        liftState = newState;
    }
    public boolean liftReady()
    {
        if( Math.abs(lift_left.getCurrentPosition() - lift_set_point) < 35){
            return true;
        }
        else
        {
            return false;
        }

    }
    public void autoUpdate() throws InterruptedException {

        switch (liftState) {
            case READY: {
                opMode.telemetry.addData("Lift State", "Ready");
                if (false) {
                    liftState = LiftState.CONE_POSITIONING;
                    positioner.positionCone();
                    stateMachineTimer.reset();
                } else if (false) {
                    previousPresetPosition();
                    liftState = LiftState.LOWERING_LIFT_TO_DROP;
                    stateMachineTimer.reset();
                }
                break;
            }
            case CONE_POSITIONING: {
                opMode.telemetry.addData("Lift State", ":Positioning Cone");
                if (stateMachineTimer.milliseconds() > 250) {
                    liftState = LiftState.LOWERING_LIFT_TO_GRAB;
                    choosePresetPosition("OneCone");
                }
                break;
            }
            case LOWERING_LIFT_TO_GRAB: {
                opMode.telemetry.addData("Lift State", "Lowering Lift to Add");
                if (liftReady()) {
                    liftState = LiftState.GRABBING_CONE;
                    gripper.grabCone();
                    stateMachineTimer.reset();
                }
                break;
            }
            case GRABBING_CONE: {
                opMode.telemetry.addData("Lift State", "Grabbing Cone");
                if (stateMachineTimer.milliseconds() > 250) {
                    liftState = LiftState.POSITIONER_RELEASING;
                    positioner.releaseCone();
                    stateMachineTimer.reset();
                }
                break;
            }
            case POSITIONER_RELEASING: {
                opMode.telemetry.addData("Lift State", "Releasing Positioner");
                if (stateMachineTimer.milliseconds() > 250) {
                    liftState = LiftState.LIFT_RAISING;
                    choosePresetPosition("LowPole");
                    stateMachineTimer.reset();
                }
                break;
            }
            case LIFT_RAISING: {
                opMode.telemetry.addData("Lift State", "Raising Lift");
                if (liftReady()) {
                    liftState = LiftState.READY;
                    stateMachineTimer.reset();
                }
                break;
            }
            case LOWERING_LIFT_TO_DROP: {
                opMode.telemetry.addData("Lift State", "Lowering Lift To Drop");
                if (liftReady()) {
                    liftState = LiftState.CONE_DROPPING;
                    gripper.dropCone();
                    stateMachineTimer.reset();
                }
                break;

            }
            case CONE_DROPPING: {
                opMode.telemetry.addData("Lift State", "Dropping Cone");
                if (stateMachineTimer.milliseconds() > 250) {
                    liftState = LiftState.READY;
                    stateMachineTimer.reset();
                }
                break;
            }
        }

        opMode.telemetry.addData("Preset Lift Position", liftTargetPreset);

        int liftActualPosition =  lift_left.getCurrentPosition();
        double error = lift_set_point - liftActualPosition;

        double derivative = (error - lastError) / PIDTimer.seconds();
        integral_sum += (error * PIDTimer.seconds());

        out = (Kp * error) + (Ki * integral_sum) + (Kd * derivative) + Kg;
        out = Range.clip(out, -0.1, 0.4);

        lift_left.setPower(out);
        lift_right.setPower(out);

        lastError = error;

        PIDTimer.reset();
    }
    public void update(Gamepad currentGamepad2, Gamepad previousGamepad2) throws InterruptedException {

        switch (liftState) {
            case READY: {
                opMode.telemetry.addData("Lift State", "Ready");
                if (currentGamepad2.a && !previousGamepad2.a && gripper.state == "Release") {
                    liftState = LiftState.CONE_POSITIONING;
                    positioner.positionCone();
                    stateMachineTimer.reset();
                } else if (currentGamepad2.b && !previousGamepad2.b && gripper.state == "Grab") {
                    previousPresetPosition();
                    liftState = LiftState.LOWERING_LIFT_TO_DROP;
                    stateMachineTimer.reset();
                }
                break;
            }
            case CONE_POSITIONING: {
                opMode.telemetry.addData("Lift State", ":Positioning Cone");
                if (stateMachineTimer.milliseconds() > 250) {
                    liftState = LiftState.LOWERING_LIFT_TO_GRAB;
                    choosePresetPosition("OneCone");
                }
                break;
            }
            case LOWERING_LIFT_TO_GRAB: {
                opMode.telemetry.addData("Lift State", "Lowering Lift to Add");
                if (liftReady()){
                    liftState = LiftState.GRABBING_CONE;
                    gripper.grabCone();
                    stateMachineTimer.reset();
                }
                break;
            }
            case GRABBING_CONE: {
                opMode.telemetry.addData("Lift State", "Grabbing Cone");
                if (stateMachineTimer.milliseconds() > 250) {
                    liftState = LiftState.POSITIONER_RELEASING;
                    positioner.releaseCone();
                    stateMachineTimer.reset();
                }
                break;
            }
            case POSITIONER_RELEASING: {
                opMode.telemetry.addData("Lift State", "Releasing Positioner");
                if (stateMachineTimer.milliseconds() > 250) {
                    liftState = LiftState.LIFT_RAISING;
                    choosePresetPosition("LowPole");
                    stateMachineTimer.reset();
                }
                break;
            }
            case LIFT_RAISING: {
                opMode.telemetry.addData("Lift State", "Raising Lift");
                if (liftReady()) {
                    liftState = LiftState.READY;
                    stateMachineTimer.reset();
                }
                break;
            }
            case LOWERING_LIFT_TO_DROP: {
                opMode.telemetry.addData("Lift State", "Lowering Lift To Drop");
                if (liftReady()) {
                    liftState = LiftState.CONE_DROPPING;
                    gripper.dropCone();
                    stateMachineTimer.reset();
                }
                break;

            }
            case CONE_DROPPING: {
                opMode.telemetry.addData("Lift State", "Dropping Cone");
                if (stateMachineTimer.milliseconds() > 250) {
                    liftState = LiftState.READY;
                    stateMachineTimer.reset();
                }
                break;
            }
        }

        if (currentGamepad2.y && !previousGamepad2.y){
            liftState = LiftState.READY;
        }
        if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up){
            nextPolePosition();
        }
        if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down){
            previousPolePosition();

        }
        if (currentGamepad2.dpad_right && !previousGamepad2.dpad_right){
            nextPresetPosition();
        }
        if (currentGamepad2.dpad_left&& !previousGamepad2.dpad_left){
            previousPresetPosition();

        }
        if(currentGamepad2.left_trigger > 0.5 && previousGamepad2.left_trigger <= 0.5){
            positioner.positionCone();
        }

        if(currentGamepad2.right_trigger > 0.5 && previousGamepad2.right_trigger <= 0.5){
            positioner.releaseCone();
        }
        if(currentGamepad2.left_bumper && !previousGamepad2.left_bumper){
            gripper.dropCone();
        }

        if(currentGamepad2.right_bumper && !previousGamepad2.right_bumper){
            gripper.grabCone();
        }

        int liftActualPosition =  lift_left.getCurrentPosition();
        double error = lift_set_point - liftActualPosition;

        double derivative = (error - lastError) / PIDTimer.seconds();
        integral_sum += (error * PIDTimer.seconds());

        out = (Kp * error) + (Ki * integral_sum) + (Kd * derivative) + Kg;
        out = Range.clip(out, -0.1, 0.4);

        lift_left.setPower(out);
        lift_right.setPower(out);

        lastError = error;

        PIDTimer.reset();
    }
}
