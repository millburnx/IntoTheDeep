package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.CommandOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.common.subsystems.ArmPID

@Config
object MOTORPOWER {
    @JvmField
    var power = 0.0
}

@TeleOp(name = "Motor Test")
class MotorTest() : CommandOpMode() {
    val motor: DcMotor by lazy {
        val motor = hardwareMap["slides"] as DcMotor
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motor.direction = DcMotorSimple.Direction.FORWARD
        return@lazy motor
    }
    val lift: ArmPID = ArmPID(hardwareMap)

    override fun initialize() {}

    override fun run() {
        lift.setTarget(155)
        lift.run()
        motor.power = MOTORPOWER.power
    }
}