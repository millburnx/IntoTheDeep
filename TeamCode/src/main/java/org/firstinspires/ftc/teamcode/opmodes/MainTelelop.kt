package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.common.subsystems.ArmPID
import org.firstinspires.ftc.teamcode.common.subsystems.DriveSubsystem
import org.firstinspires.ftc.teamcode.common.subsystems.LiftPID
import org.firstinspires.ftc.teamcode.common.utils.PoseColor
import org.firstinspires.ftc.teamcode.common.utils.Telemetry
import java.util.ArrayList

@Config
@TeleOp(name = "MainTeleOp")
class MainTelelop : CommandOpMode() {
    private var drive: DriveSubsystem? = null
    var tel: Telemetry? = null
    var gamepad: GamepadEx? = null
    var pos: Pose2d = Pose2d()
    var dash: FtcDashboard = FtcDashboard.getInstance()
    var arm: ArmPID? = null
    var lift: LiftPID? = null

    override fun initialize() {
        drive = DriveSubsystem(hardwareMap, -1.0)
        //        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        tel = Telemetry()
        dash = FtcDashboard.getInstance()
        lift = LiftPID(hardwareMap)
        arm = ArmPID(hardwareMap, lift!!.lift::getCurrentPosition)
    }


    override fun run() {
        super.run()

        if (gamepad1.b) {
            println("RESETTING IMU")
            drive!!.imu.resetYaw()
        }

        if (gamepad1.dpad_down) {
            arm!!.target = ArmPID.floor
        } else if (gamepad1.dpad_up) {
            arm!!.target = ArmPID.up
        }
        if (gamepad1.triangle) {
            lift!!.target = 2000
        } else if (gamepad1.circle) {
            lift!!.target = 1000
        } else if (gamepad1.cross) {
            lift!!.target = 0
        }

        arm!!.run(telemetry)
        lift!!.run()

        val power = -gamepad1.left_stick_y.toDouble()
        val strafe = gamepad1.left_stick_x.toDouble()
        val turn = gamepad1.right_stick_x.toDouble()

        if (!field) {
            drive!!.robotCentric(power, strafe * 1.1, turn)
        } else {
            val heading = drive!!.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)
            telemetry.addData("heading", Math.toDegrees(heading))
            drive!!.fieldCentric(if (yflip) -power else power, strafe * 1.1, turn, heading + Math.toRadians(90.0))
        }
        drive!!.updatePos()

        pos = drive!!.getPos()

        telemetry.addData("left", drive!!.leftOdom.position)
        telemetry.addData("center", drive!!.centerOdom.position)
        telemetry.addData("right", drive!!.rightOdom.position)
        val list: MutableList<PoseColor> = ArrayList<PoseColor>()
        list.add(PoseColor(pos, "#0000ff"))
        tel!!.drawField(list, dash)
        telemetry.update()
    }

    companion object {
        var field: Boolean = false

        var yflip: Boolean = false
    }
}
