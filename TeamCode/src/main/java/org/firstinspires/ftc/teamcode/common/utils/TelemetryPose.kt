package org.firstinspires.ftc.teamcode.common.utils

import com.arcrobotics.ftclib.geometry.Pose2d

data class TelemetryPose(val pose: Pose2d, val color: String, val size: Double? = null)