package com.millburnx.dashboard.canvas

interface FTCDashboard {
    fun sendTelemetryPacket(telemetryPacket: TelemetryPacket)
}