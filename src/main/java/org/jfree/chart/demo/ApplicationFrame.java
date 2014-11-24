/* ===========================================================
 * JFreeChart : a free chart library for the Java(tm) platform
 * ===========================================================
 *
 * (C) Copyright 2000-2014, by Object Refinery Limited and Contributors.
 *
 * Project Info:  http://www.jfree.org/jfreechart/index.html
 *
 * This library is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 2.1 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public
 * License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
 * USA.
 *
 * [Oracle and Java are registered trademarks of Oracle and/or its affiliates.
 * Other names may be trademarks of their respective owners.]
 *
 * ---------------------
 * ApplicationFrame.java
 * ---------------------
 * (C) Copyright 2000-2014, by Object Refinery Limited.
 *
 * Original Author:  David Gilbert (for Object Refinery Limited);
 * Contributor(s):   -;
 *
 * Changes (from 30-May-2002)
 * --------------------------
 * 30-May-2002 : Added title (DG);
 * 13-Oct-2002 : Fixed errors reported by Checkstyle (DG);
 * 14-Jun-2012 : Moved from JCommon to JFreeChart (DG);
 * 19-Mar-2014 : Move to demo package (DG);
 * 
 */

package org.jfree.chart.demo;

import java.awt.event.WindowEvent;
import java.awt.event.WindowListener;
import javax.swing.JFrame;

/**
 * A frame for simple demo applications (the frame shuts down the JVM when
 * the window is closed).
 */
public class ApplicationFrame extends JFrame implements WindowListener {

    private static final long serialVersionUID = 1L;

    /**
     * Constructs a new application frame.
     *
     * @param title  the frame title.
     */
    public ApplicationFrame(String title) {
        super(title);
        addWindowListener(this);
    }

    /**
     * Listens for the main window closing, and shuts down the application.
     *
     * @param event  information about the window event.
     */
    @Override
    public void windowClosing(WindowEvent event) {
        if (event.getWindow() == this) {
            dispose();
            System.exit(0);
        }
    }

    /**
     * Required for WindowListener interface, but not used by this class.
     *
     * @param event  information about the window event.
     */
    @Override
    public void windowClosed(WindowEvent event) {
        // ignore
    }

    /**
     * Required for WindowListener interface, but not used by this class.
     *
     * @param event  information about the window event.
     */
    @Override
    public void windowActivated(WindowEvent event) {
        // ignore
    }

    /**
     * Required for WindowListener interface, but not used by this class.
     *
     * @param event  information about the window event.
     */
    @Override
    public void windowDeactivated(WindowEvent event) {
        // ignore
    }

    /**
     * Required for WindowListener interface, but not used by this class.
     *
     * @param event  information about the window event.
     */
    @Override
    public void windowDeiconified(WindowEvent event) {
        // ignore
    }

    /**
     * Required for WindowListener interface, but not used by this class.
     *
     * @param event  information about the window event.
     */
    @Override
    public void windowIconified(WindowEvent event) {
        // ignore
    }

    /**
     * Required for WindowListener interface, but not used by this class.
     *
     * @param event  information about the window event.
     */
    @Override
    public void windowOpened(WindowEvent event) {
        // ignore
    }

}
