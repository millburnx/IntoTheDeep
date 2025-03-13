package org.firstinspires.ftc.teamcode.common.commands.drive

import com.acmerobotics.dashboard.config.Config

@Config
open class PickupPIDSettings : PIDSettings() {
    companion object {
        @JvmField
        var kP: Double = 0.2

        @JvmField
        var kI: Double = 0.0

        @JvmField
        var kD: Double = 0.0

        @JvmField
        var kPHeading: Double = 0.2

        @JvmField
        var kIHeading: Double = 0.0

        @JvmField
        var kDHeading: Double = 0.0

        @JvmField
        var tolerance: Double = 1.0

        @JvmField
        var headingTolerance: Double = 2.0

        @JvmField
        var usePowerSettling: Boolean = false

        @JvmField
        var wheelThreshold: Double = 0.15

        @JvmField
        var squid: Boolean = false

        @JvmField
        var squidv2: Boolean = false
    }
}

@Config
open class PIDSettings {
    companion object {
        @JvmField
        var kP: Double = 0.175

        @JvmField
        var kI: Double = 0.0

        @JvmField
        var kD: Double = 0.005

        @JvmField
        var kPHeading: Double = 0.075

        @JvmField
        var kIHeading: Double = 0.0

        @JvmField
        var kDHeading: Double = 0.0

        @JvmField
        var tolerance: Double = 1.0

        @JvmField
        var headingTolerance: Double = 2.0

        @JvmField
        var usePowerSettling: Boolean = false

        @JvmField
        var wheelThreshold: Double = 0.15

        @JvmField
        var squid: Boolean = false

        @JvmField
        var squidv2: Boolean = false
    }
}
