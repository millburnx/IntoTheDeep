package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.utils.OpMode

@TeleOp(name = "Camera Debugger", group = "Tuning")
class CameraDebugger : OpMode() {
    override val robot by lazy { CameraRobot(this) }

    override fun exec() {
//        robot.telemetry.addData()
    }
}
