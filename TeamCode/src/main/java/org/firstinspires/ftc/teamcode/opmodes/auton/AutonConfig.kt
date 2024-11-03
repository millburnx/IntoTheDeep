package org.firstinspires.ftc.teamcode.opmodes.auton

import com.acmerobotics.dashboard.config.Config

@Config
object AutonConfig {
    @JvmField
    var multiF = 0.75

    @JvmField
    var multiH = 0.75

    @JvmField
    var pathName = "blue"

    @JvmField
    var useEndingHeading = true

    @JvmField
    var endingHeading = 180.0

    @JvmField
    var pidT_kP = 0.1

    @JvmField
    var pidT_kI = 0.0

    @JvmField
    var pidT_kD = 0.005

    @JvmField
    var pidH_kP = 0.025

    @JvmField
    var pidH_kI = 0.0

    @JvmField
    var pidH_kD = 0.0

    @JvmField
    var minRange = 12.0

    @JvmField
    var maxRange = 16.0

    @JvmField
    var threshold = 0.75

    @JvmField
    var headingTolerance = 3.0

    @JvmField
    var segments: Int = 1

    @JvmField
    var timeout: Long = 5000

    @JvmField
    var barPower: Double = 0.3
}