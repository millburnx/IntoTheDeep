package org.firstinspires.ftc.teamcode.common.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class LiftPID {
    public DcMotorEx rotate;

    private PIDController controller;
    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static int target = 0;

    private final double ticks_in_degree = 8192/360.0;

    public LiftPID(HardwareMap hardwareMap) {
        controller = new PIDController(p, i, d);
        rotate = hardwareMap.get(DcMotorEx.class, "rotate");
        rotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void run() {
        controller.setPID(p, i, d);
        int pos = rotate.getCurrentPosition();
        double pid = controller.calculate(pos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid + ff;

        rotate.setPower(-power);
    }

    public void setTarget(int targetPos) {
        target = targetPos;
    }
}