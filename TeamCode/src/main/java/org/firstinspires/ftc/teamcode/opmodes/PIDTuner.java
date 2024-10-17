package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.subsystems.ArmPID;
import org.firstinspires.ftc.teamcode.common.subsystems.LiftPID;

@Config
@TeleOp
public class PIDTuner extends OpMode {
    public ArmPID arm;
    public LiftPID lift;

    public static int armTarget = 0;
    public static int liftTarget = 0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        arm = new ArmPID(hardwareMap);
        lift = new LiftPID(hardwareMap);
    }

    @Override
    public void loop() {
        arm.setTarget(armTarget);
        arm.run();

        lift.setTarget(liftTarget);
        lift.run();

        telemetry.addData("arm pos: ", arm.rightRotate.getCurrentPosition());
        telemetry.addData("arm target: ", armTarget);

        telemetry.addData("lift pos: ", lift.lift.getCurrentPosition());
        telemetry.addData("lift target; ", liftTarget);

    }
}