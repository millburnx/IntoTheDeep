package com.millburnx.pathplanner

import com.millburnx.pathplanner.ui.Theme
import com.millburnx.pathplanner.ui.layout.StackLayout
import java.awt.Dimension
import javax.swing.JFrame
import javax.swing.LookAndFeel
import javax.swing.UIManager

fun main(args: Array<String>) {
    val scale = getScale(args)
    setScale(scale)
    setLAF()
    val frame = getJFrame()
    val stackLayout = StackLayout()
    stackLayout.addTo(frame)
}

fun getScale(args: Array<String>): Double {
    val flag = "--scale="
    for (arg in args) {
        if (arg.startsWith(flag)) {
            return arg.substring(flag.length).toDouble()
        }
    }
    return 1.0
}

fun setScale(scale: Double) {
    System.setProperty("flatlaf.uiScale", "$scale")
    Theme.scale = scale
}

fun setLAF() {
    try {
        UIManager.setLookAndFeel(Theme.LAF as LookAndFeel)
    } catch (e: Exception) {
        e.printStackTrace()
        System.err.println("Failed to initialize LaF, ui will look weird")
    }
}

fun getJFrame(): JFrame {
    val frame = JFrame("Pure Pursuit")
    frame.defaultCloseOperation = JFrame.EXIT_ON_CLOSE
    frame.size = Dimension(600, 800)
    frame.setLocationRelativeTo(null)
    frame.isVisible = true
    return frame
}