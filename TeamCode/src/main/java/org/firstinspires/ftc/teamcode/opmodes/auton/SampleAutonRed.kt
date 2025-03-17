package org.firstinspires.ftc.teamcode.opmodes.auton

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.Autonomous

@Config
@Autonomous(name = "Sample Auton Red", preselectTeleOp = "Main Teleop Red")
class SampleAutonRed : SampleAuton() {
    override fun initialize() {
        robot.isRed = true
        super.initialize()
    }
}
