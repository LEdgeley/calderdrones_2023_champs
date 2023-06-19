package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Gripper {
    private Servo gripper;
    public static double GRIPPER_OPEN = 0.45;
    public static double GRIPPER_CLOSED = 0.3;
    public String state = "Grab";

    public Gripper(HardwareMap hardwareMap){
        gripper = hardwareMap.get(Servo.class, "gripper");
    }
    public void grabCone()  {
        gripper.setPosition(GRIPPER_OPEN);
        state="Grab";
    }
    public void dropCone() {
        gripper.setPosition(GRIPPER_CLOSED);
        state="Release";
    }

    public double getPosition(){
        return gripper.getPosition();
    }

}
