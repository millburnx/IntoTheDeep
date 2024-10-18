package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystems.ArmPID;
import org.firstinspires.ftc.teamcode.common.subsystems.LiftPID;
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
    ArmPID arm;
    LiftPID lift;

    public static boolean field = false;

    public static boolean yflip = false;


    @Override
    public void initialize() {
        drive = new DriveSubsystem(hardwareMap, -1);
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        tel = new Telemetry();
        dash = FtcDashboard.getInstance();
        arm = new ArmPID(hardwareMap);
        lift = new LiftPID(hardwareMap);
    }


    @Override
    public void run() {
        super.run();

        if (gamepad1.b) {
            System.out.println("RESETTING IMU");
            drive.imu.resetYaw();
        }

        if (gamepad1.dpad_down) {
            arm.setTarget(ArmPID.floor);
        } else if (gamepad1.dpad_up) {
            arm.setTarget(ArmPID.up);
        }
        if (gamepad1.triangle) {
            lift.setTarget(2000);
        } else if (gamepad1.circle) {
            lift.setTarget(1000);
        } else if (gamepad1.cross) {
            lift.setTarget(0);
        }

        arm.run();
        lift.run();

        double power = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        if (!field) {
            drive.robotCentric(power, strafe * 1.1, turn);
        } else {
            double heading = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            telemetry.addData("heading", Math.toDegrees(heading));
            drive.fieldCentric(yflip ? -power : power, strafe * 1.1, turn, heading + Math.toRadians(90));
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
