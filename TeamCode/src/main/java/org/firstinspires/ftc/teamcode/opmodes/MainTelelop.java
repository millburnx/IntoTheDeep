package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.common.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.common.utils.PoseColor;
import org.firstinspires.ftc.teamcode.common.utils.Telemetry;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp(name = "MainTeleOp")
public class MainTelelop extends CommandOpMode {
    private DriveSubsystem drive;
    Telemetry tel;
    GamepadEx gamepad;
    Pose2d pos;
    FtcDashboard dash;

    public static boolean field = false;

    public static boolean yflip = false;

    @Override
    public void initialize() {
        drive = new DriveSubsystem(hardwareMap, -1);
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        tel = new Telemetry();
        dash = FtcDashboard.getInstance();
    }


    @Override
    public void run() {
        super.run();

        double power = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        if (!field) {
            drive.robotCentric(power, strafe * 1.1, turn);
        } else {
            drive.fieldCentric(yflip ? -power : power, strafe * 1.1, turn, drive.getPos().getHeading());
        }
        drive.updatePos();

        pos = drive.getPos();

        telemetry.addData("left", drive.leftOdom.getPosition());
        telemetry.addData("center", drive.centerOdom.getPosition());
        telemetry.addData("right", drive.rightOdom.getPosition());
        List<PoseColor> list = new ArrayList<>();
        list.add(new PoseColor(pos, "#0000ff"));
        tel.drawField(list, dash);
        telemetry.update();
    }
}
