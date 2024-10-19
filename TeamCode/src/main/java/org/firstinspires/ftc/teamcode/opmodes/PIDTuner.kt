package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.subsystems.ArmPID
import org.firstinspires.ftc.teamcode.common.subsystems.LiftPID

@Config
@TeleOp
class PIDTuner : OpMode() {
    val arm: ArmPID by lazy {
        ArmPID(hardwareMap)
    }
    val lift: LiftPID by lazy {
        LiftPID(hardwareMap)
    }

    override fun init() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
    }

    override fun loop() {
        arm.target = armTarget
        arm.run()

        lift.target = liftTarget
        lift.run()

        telemetry.addData("arm pos: ", arm.rightRotate.currentPosition)
        telemetry.addData("arm angle: ", arm)
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