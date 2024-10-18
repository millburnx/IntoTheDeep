package org.firstinspires.ftc.teamcode.common.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class LiftPID {
    public DcMotorEx lift;

    private PIDController controller;
    public static double p = 0.02, i = 0, d = 0;
    public static double f = 0;

    public int target = 10;

    public static double ticks_in_degree = 8192.0 / 360.0;

    public LiftPID(HardwareMap hardwareMap) {
        controller = new PIDController(p, i, d);
        lift = hardwareMap.get(DcMotorEx.class, "slides");
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void run() {
        controller.setPID(p, i, d);
        int pos = lift.getCurrentPosition();
        double pid = controller.calculate(pos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid + ff;

        lift.setPower(power);
    }

    public void setTarget(int targetPos) {
        target = targetPos;
    }
}