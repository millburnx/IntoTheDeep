package org.firstinspires.ftc.teamcode.common.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class LiftPID {
    public DcMotorEx leftRotate;
    public DcMotorEx rightRotate;

    private PIDController controller;
    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static int target = 0;

    public static double ticks_in_degree = 8192 / 360.0;

    public LiftPID(HardwareMap hardwareMap) {
        controller = new PIDController(p, i, d);
        leftRotate = hardwareMap.get(DcMotorEx.class, "leftRotate");
        leftRotate.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightRotate = hardwareMap.get(DcMotorEx.class, "rightRotate");
        rightRotate.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void run() {
        controller.setPID(p, i, d);
        int pos = leftRotate.getCurrentPosition();
        double pid = controller.calculate(pos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid + ff;

        leftRotate.setPower(-power);
        rightRotate.setPower(-power);
    }

    public void setTarget(int targetPos) {
        target = targetPos;
    }
}