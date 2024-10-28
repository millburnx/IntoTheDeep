package com.millburnx.pathPlanner.layout

import java.awt.BorderLayout
import javax.swing.JPanel

public class MainLayout : JPanel() {
    public val navbar: Navbar = Navbar()
    public val viewer: ViewWrapper = ViewWrapper()
    public val editor: EditorWrapper = EditorWrapper()

    init {
        layout = BorderLayout()
        add(navbar, BorderLayout.NORTH)
        add(viewer, BorderLayout.CENTER)
        add(editor, BorderLayout.SOUTH)
    }
}