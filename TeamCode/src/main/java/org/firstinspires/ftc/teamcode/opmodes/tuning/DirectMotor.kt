package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.subsystems.Arm
import org.firstinspires.ftc.teamcode.common.subsystems.Drive
import org.firstinspires.ftc.teamcode.common.subsystems.Lift
import org.firstinspires.ftc.teamcode.common.utils.Telemetry

@Config
@TeleOp(name = "Direct Motor")
class DirectMotor : CommandOpMode() {
    val multiTelem: MultipleTelemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
    val drive: Drive by lazy { Drive(hardwareMap, Telemetry(), FtcDashboard.getInstance()) }
    val lift: Lift by lazy { Lift(hardwareMap) }
    val arm: Arm by lazy { Arm(hardwareMap, multiTelem, lift::position) }

    override fun initialize() {
    }

    override fun run() {
        super.run()

        drive.leftFront.set(driveFrontLeft)
        drive.rightFront.set(driveFrontRight)
        drive.leftRear.set(driveBackLeft)
        drive.rightRear.set(driveBackRight)

        multiTelem.addData("frontLeft", drive.leftFront.currentPosition)
        multiTelem.addData("frontRight", drive.rightFront.currentPosition)
        multiTelem.addData("backLeft", drive.leftRear.currentPosition)
        multiTelem.addData("backRight", drive.rightRear.currentPosition)

        arm.setPower(armPower)
        multiTelem.addData("arm", arm.position)

        lift.setPower(liftPower)
        multiTelem.addData("lift", lift.position)
        multiTelem.update()
    }

    companion object {
        @JvmField
        var driveFrontLeft: Double = 0.0

        @JvmField
        var driveFrontRight: Double = 0.0

        @JvmField
        var driveBackLeft: Double = 0.0

        @JvmField
        var driveBackRight: Double = 0.0

        @JvmField
        var armPower: Double = 0.0

        @JvmField
        var liftPower: Double = 0.0
    }
}