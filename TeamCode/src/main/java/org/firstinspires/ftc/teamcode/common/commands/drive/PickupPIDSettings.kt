package org.firstinspires.ftc.teamcode.common.commands.drive

import com.acmerobotics.dashboard.config.Config

@Config
class PickupPIDSettings : PIDSettings() {
    companion object {
        @JvmField
        var kP: Double = 0.0

        @JvmField
        var kI: Double = 0.0

        @JvmField
        var kD: Double = 0.0

        @JvmField
        var kPHeading: Double = 0.0

        @JvmField
        var kIHeading: Double = 0.0

        @JvmField
        var kDHeading: Double = 0.0

        @JvmField
        var tolerance: Double = 1.0

        @JvmField
        var headingTolerance: Double = 4.0

        @JvmField
        var usePowerSettling: Boolean = false

        @JvmField
        var wheelThreshold: Double = 0.15
    }
}

@Config
open class PIDSettings {
    companion object {
        @JvmField
        var kP: Double = 0.22

        @JvmField
        var kI: Double = 0.0

        @JvmField
        var kD: Double = 0.0285

        @JvmField
        var kPHeading: Double = 0.0875

        @JvmField
        var kIHeading: Double = 0.0

        @JvmField
        var kDHeading: Double = 0.00001

        @JvmField
        var tolerance: Double = 1.0

        @JvmField
        var headingTolerance: Double = 2.0

        @JvmField
        var usePowerSettling: Boolean = false

        @JvmField
        var wheelThreshold: Double = 0.15
    }
}
