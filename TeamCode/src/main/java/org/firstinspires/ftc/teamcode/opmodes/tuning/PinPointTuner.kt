package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.utils.EdgeDetector
import org.firstinspires.ftc.teamcode.common.utils.OpMode

@TeleOp(name = "PinPoint Tuner")
class PinPointTuner : OpMode() {
    override val robot: Robot by lazy { Robot(this) }

    override fun initialize() {
        super.initialize()
        triggers

        robot.telemetry.addData("Status", "Initialized")
        robot.telemetry.addData("X offset", robot.drive.pinPoint.pinPoint.xOffset)
        robot.telemetry.addData("Y offset", robot.drive.pinPoint.pinPoint.yOffset)
        robot.telemetry.addData("Device Version Number:", robot.drive.pinPoint.pinPoint.deviceVersion)
        robot.telemetry.addData("Device Scalar", robot.drive.pinPoint.pinPoint.yawScalar)
        robot.telemetry.update()
    }

    val triggers by lazy {
        object {
            val reset = EdgeDetector(gamepad1::a) { robot.drive.pinPoint.reset() }
            val resetImu = EdgeDetector(gamepad1::b) { robot.drive.pinPoint.resetImu() }
        }
    }

    override fun exec() {
        robot.drive.fieldCentric(
            gamepad1.left_stick_y.toDouble(),
            -gamepad1.left_stick_x.toDouble(),
            -gamepad1.right_stick_x.toDouble(),
            0.0,
        )

        robot.telemetry.addData("Position", robot.drive.pose.toString())
        robot.telemetry.addData("Velocity", robot.drive.velocity.toString())

        // READY: the device is working as normal
        // CALIBRATING: the device is calibrating and outputs are put on hold
        // NOT_READY: the device is resetting from scratch. This should only happen after a power-cycle
        // FAULT_NO_PODS_DETECTED - the device does not detect any pods plugged in
        // FAULT_X_POD_NOT_DETECTED - The device does not detect an X pod plugged in
        // FAULT_Y_POD_NOT_DETECTED - The device does not detect a Y pod plugged in
        robot.telemetry.addData("Status", robot.drive.pinPoint.status)
        robot.telemetry.addData("Pinpoint Hertz", robot.drive.pinPoint.hertz)
        robot.telemetry.addData(
            "REV Hub Hertz: ",
            1 / robot.deltaTime.deltaTime,
        )
    }
}
