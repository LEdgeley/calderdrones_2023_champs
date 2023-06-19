package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Positioner {
    private Servo left_positioner;
    private Servo right_positioner;
    public static double positioning_arm_left_open = 0.35;
    public static double positioning_arm_left_closed = 0.265;
    public static double positioning_arm_right_open = 0.60;
    public static double positioning_arm_right_closed = 0.715;

    public Positioner(HardwareMap hardwareMap){
        left_positioner = hardwareMap.get(Servo.class, "left_positioning_arm");
        right_positioner = hardwareMap.get(Servo.class, "right_positioning_arm");
    }
    public void positionCone() throws InterruptedException {
        left_positioner.setPosition(positioning_arm_left_closed);
        right_positioner.setPosition(positioning_arm_right_closed);
    }
    public void releaseCone() throws InterruptedException {
        left_positioner.setPosition(positioning_arm_left_open);
        right_positioner.setPosition(positioning_arm_right_open);
    }

    public double[] getPosition(){
        double[] result = new double[2];
        result[0] = left_positioner.getPosition();
        result[1] = right_positioner.getPosition();
        return result;

    }
}
