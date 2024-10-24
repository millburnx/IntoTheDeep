package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.CommandOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.common.subsystems.Arm

@Config
object MOTORPOWER {
    @JvmField
    var power = 0.0
}

@TeleOp(name = "Motor Test")
class MotorTest() : CommandOpMode() {
    lateinit var motor: DcMotor;
    lateinit var lift: Arm

    override fun initialize() {
        motor = hardwareMap.get(DcMotor::class.java, "slides")
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motor.direction = DcMotorSimple.Direction.FORWARD

        lift = Arm(hardwareMap, telemetry) { 0 }
    }

    override fun run() {
        super.run()
        lift.target = 155
        motor.power = MOTORPOWER.power
    }
}