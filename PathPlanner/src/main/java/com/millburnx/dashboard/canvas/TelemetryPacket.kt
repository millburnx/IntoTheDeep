package com.millburnx.dashboard.canvas

interface TelemetryPacket {
    fun addLine(line: String)
    fun put(key: String, value: Any)
    fun putAll(map: Map<String, Any>)
    fun clearLines()
    fun addTimestamp(): Long
    fun fieldOverlay(): Canvas
}