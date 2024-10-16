package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.common.subsystems.LiftPID;

@Config
@TeleOp
public class LiftPIDTuner extends OpMode {
    public LiftPID lift;

    public static int target = 0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        lift = new LiftPID(hardwareMap);
    }

    @Override
    public void loop() {
        lift.setTarget(target);
        lift.run();

        telemetry.addData("r pos: ", lift.rightRotate.getCurrentPosition());
        telemetry.addData("l pos: ", lift.leftRotate.getCurrentPosition());
        telemetry.addData("r angle:", lift.rightRotate.getCurrentPosition() / LiftPID.ticks_in_degree);
        telemetry.addData("l angle:", lift.leftRotate.getCurrentPosition() / LiftPID.ticks_in_degree);
        telemetry.addData("target: ", target);
    }
}