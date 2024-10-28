package com.millburnx.pathPlanner

import com.formdev.flatlaf.FlatLaf
import com.formdev.flatlaf.intellijthemes.FlatOneDarkIJTheme
import com.millburnx.pathPlanner.layout.StackLayout
import java.awt.Dimension
import javax.swing.JFrame
import javax.swing.LookAndFeel
import javax.swing.UIManager

public fun main(args: Array<String>) {
    val scale = getScale(args)
    setScale(scale)
    setLAF()
    val frame = createJFrame()
    val stackLayout = StackLayout()
    stackLayout.addTo(frame)
}

public fun getScale(args: Array<String>): Double {
    val flag = "--scale="
    for (arg in args) {
        if (arg.startsWith(flag)) {
            return arg.substring(flag.length).toDouble()
        }
    }
    return 1.0
}

public object Theme {
    public val LAF: FlatLaf = FlatOneDarkIJTheme()
    public var scale: Double = 1.0
}

public fun setScale(scale: Double) {
    System.setProperty("flatlaf.uiScale", "$scale")
    Theme.scale = scale
}

public fun setLAF() {
    try {
        UIManager.setLookAndFeel(Theme.LAF as LookAndFeel)
    } catch (e: Exception) {
        e.printStackTrace()
        System.err.println("Failed to initialize LaF, ui will look weird")
    }
}

public fun createJFrame(): JFrame {
    val frame = JFrame("Pure Pursuit")
    frame.defaultCloseOperation = JFrame.EXIT_ON_CLOSE
    frame.size = Dimension(600, 800)
    frame.setLocationRelativeTo(null)
    frame.isVisible = true
    return frame
}