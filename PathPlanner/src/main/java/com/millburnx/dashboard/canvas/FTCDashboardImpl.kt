package com.millburnx.dashboard.canvas

import java.awt.Color
import java.awt.Graphics2D

class FTCDashboardImpl : FTCDashboard {
    class CanvasCtx(val g2d: Graphics2D) {
        var alpha: Double = 1.0

        var fill: Color = Color.BLACK
            get() = Color(fill.red, fill.green, fill.blue, (alpha * 255).toInt())

        var stroke: Color = Color.BLACK
            get() = Color(stroke.red, stroke.green, stroke.blue, (alpha * 255).toInt())
    }

    override fun sendTelemetryPacket(telemetryPacket: TelemetryPacket) {
        TODO("Not yet implemented")
    }
}