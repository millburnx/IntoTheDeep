package org.firstinspires.ftc.teamcode.opmodes.auton

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.Autonomous

@Config
@Autonomous(name = "Specimen Auton Push Red", preselectTeleOp = "Main Teleop Red")
class SpecimenAutonPushRed : SpecimenAutonPush() {
    override fun initialize() {
        robot.isRed = true
        super.initialize()
    }
}
