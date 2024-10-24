package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.subsystems.Arm
import org.firstinspires.ftc.teamcode.common.subsystems.Lift

@Config
@TeleOp
class PIDTuner : OpMode() {
    val arm: Arm by lazy {
        Arm(hardwareMap, lift.lift::getCurrentPosition)
    }
    val lift: Lift by lazy {
        Lift(hardwareMap)
    }

    override fun init() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
    }

    override fun loop() {
        arm.target = armTarget
        arm.run(telemetry)

        lift.target = liftTarget
        lift.run()

        telemetry.addData("arm pos: ", arm.rightRotate.currentPosition + Arm.starting_ticks)
        telemetry.addData("arm angle: ", arm.angle)
        telemetry.addData("arm target: ", armTarget)

        telemetry.addData("lift pos: ", lift.lift.currentPosition)
        telemetry.addData("lift target; ", liftTarget)
    }

    companion object {
        @JvmField
        var armTarget: Int = 0

        @JvmField
        var liftTarget: Int = 0
    }
}