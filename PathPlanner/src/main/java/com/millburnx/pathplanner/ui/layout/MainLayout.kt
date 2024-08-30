package com.millburnx.pathplanner.ui.layout

import java.awt.BorderLayout
import javax.swing.JPanel

class MainLayout : JPanel() {
    val navbar = Navbar()
    val viewer = ViewWrapper()
    val editor = EditorWrapper()

    init {
        layout = BorderLayout()
        add(navbar, BorderLayout.NORTH)
        add(viewer, BorderLayout.CENTER)
        add(editor, BorderLayout.SOUTH)
    }
}
