package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.subsystems.Arm
import org.firstinspires.ftc.teamcode.common.subsystems.Lift

@Config
@TeleOp(name = "PID Tuner", group = "Tuning")
class PIDTuner : CommandOpMode() {
    val telem = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

    val arm: Arm by lazy {
        Arm(hardwareMap, telem, lift.lift::getCurrentPosition)
    }
    val lift: Lift by lazy {
        Lift(hardwareMap)
    }

    override fun initialize() {
        arm.on()
    }

    override fun run() {
        super.run()
        arm.target = armTarget.toDouble()
        lift.target = liftTarget.toDouble()

        telem.addData("arm pos: ", arm.rightRotate.currentPosition + Arm.starting_ticks)
        telem.addData("arm angle: ", arm.angle)
        telem.addData("arm target: ", armTarget)

        telem.addData("lift pos: ", lift.lift.currentPosition)
        telem.addData("lift target; ", liftTarget)
        telem.update()
    }

    companion object {
        @JvmField
        var armTarget: Int = 0

        @JvmField
        var liftTarget: Int = 0
    }
}