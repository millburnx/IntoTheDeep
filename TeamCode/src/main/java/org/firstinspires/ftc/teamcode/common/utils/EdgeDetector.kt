package org.firstinspires.ftc.teamcode.common.utils

class EdgeDetector(val button: () -> Boolean, val rising: () -> Unit = {}, val falling: () -> Unit = {}) : Subsystem() {
    var last: Boolean = false
    override fun periodic() {
        val current = button()
        if (current && !last) {
            rising()
        } else if (!current && last) {
            falling()
        }
        last = current
    }
}