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
    var pidT_kP = 0.099

    @JvmField
    var pidT_kI = 0.198

    @JvmField
    var pidT_kD = 0.0198

    @JvmField
    var pidH_kP = 0.03

    @JvmField
    var pidH_kI = 0.06

    @JvmField
    var pidH_kD = 0.00001

    @JvmField
    var minRange = 12.0

    @JvmField
    var maxRange = 16.0

    @JvmField
    var threshold = 1.0

    @JvmField
    var headingTolerance = 3.0

    @JvmField
    var segments: Int = 1

    @JvmField
    var timeout: Long = 5000

    @JvmField
    var barPower: Double = 0.3
}