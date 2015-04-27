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
 * XYBubbleRenderer.java
 * ---------------------
 * (C) Copyright 2003-2014, by Object Refinery Limited.
 *
 * Original Author:  David Gilbert (for Object Refinery Limited);
 * Contributor(s):   Christian W. Zuckschwerdt;
 *
 * Changes
 * -------
 * 28-Jan-2003 : Version 1 (DG);
 * 25-Mar-2003 : Implemented Serializable (DG);
 * 01-May-2003 : Modified drawItem() method signature (DG);
 * 30-Jul-2003 : Modified entity constructor (CZ);
 * 20-Aug-2003 : Implemented Cloneable and PublicCloneable (DG);
 * 16-Sep-2003 : Changed ChartRenderingInfo --> PlotRenderingInfo (DG);
 * 10-Feb-2004 : Small change to drawItem() method to make cut-and-paste
 *               overriding easier (DG);
 * 15-Jul-2004 : Switched getZ() and getZValue() methods (DG);
 * 19-Jan-2005 : Now accesses only primitives from dataset (DG);
 * 28-Feb-2005 : Modify renderer to use circles in legend (DG);
 * 17-Mar-2005 : Fixed bug in bubble bounds calculation (DG);
 * 20-Apr-2005 : Use generators for legend tooltips and URLs (DG);
 * ------------- JFREECHART 1.0.x ---------------------------------------------
 * 13-Dec-2005 : Added support for item labels (bug 1373371) (DG);
 * 20-Jan-2006 : Check flag for drawing item labels (DG);
 * 21-Sep-2006 : Respect the outline paint and stroke settings (DG);
 * 24-Jan-2007 : Added new equals() override (DG);
 * 06-Feb-2007 : Fixed bug 1086307, crosshairs with multiple axes (DG);
 * 20-Apr-2007 : Updated getLegendItem() for renderer change (DG);
 * 17-May-2007 : Set datasetIndex and seriesIndex in getLegendItem() (DG);
 * 18-May-2007 : Set dataset and seriesKey for LegendItem (DG);
 * 13-Jun-2007 : Fixed seriesVisibility bug (DG);
 * 17-Jun-2008 : Apply legend shape, font and paint attributes (DG);
 * 17-Jun-2012 : Remove JCommon dependencies (DG);
 *
 */

package org.jfree.chart.renderer.xy;

import java.awt.Graphics2D;
import java.awt.Paint;
import java.awt.RadialGradientPaint;
import java.awt.Shape;
import java.awt.Stroke;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;

import org.jfree.chart.LegendItem;
import org.jfree.chart.axis.ValueAxis;
import org.jfree.chart.ui.RectangleEdge;
import org.jfree.chart.util.PublicCloneable;
import org.jfree.chart.entity.EntityCollection;
import org.jfree.chart.plot.CrosshairState;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.chart.plot.PlotRenderingInfo;
import org.jfree.chart.plot.XYPlot;
import org.jfree.chart.util.ParamChecks;
import org.jfree.data.Range;
import org.jfree.data.xy.XYDataset;
import org.jfree.data.xy.XYZDataset;

/**
 * A renderer that draws a circle at each data point with a diameter that is
 * determined by the z-value in the dataset (the renderer requires the dataset
 * to be an instance of {@link XYZDataset}).  The example shown here
 * is generated by the <code>XYBubbleChartDemo1.java</code> program
 * included in the JFreeChart demo collection:
 * <br><br>
 * <img src="../../../../../images/XYBubbleRendererSample.png"
 * alt="XYBubbleRendererSample.png">
 */
public class XYBubbleRenderer extends AbstractXYItemRenderer
        implements XYItemRenderer, PublicCloneable {

    /** For serialization. */
    public static final long serialVersionUID = -5221991598674249125L;

    /** Controls how the width and height of the bubble are scaled. */
    private ScaleType scaleType;

    /** 
     * A flag that controls whether or not outlines are drawn around the 
     * bubbles. 
     */
    private boolean drawOutlines;
    
    /**
     * Constructs a new renderer.
     */
    public XYBubbleRenderer() {
        this(ScaleType.BOTH_AXES);
    }

    /**
     * Constructs a new renderer with the specified type of scaling.
     *
     * @param scaleType  the type of scaling (<code>null</code> not permitted).
     */
    public XYBubbleRenderer(ScaleType scaleType) {
        super();
        ParamChecks.nullNotPermitted(scaleType, "scaleType");
        this.scaleType = scaleType;
        this.drawOutlines = false;
        setDefaultLegendShape(new Ellipse2D.Double(-4.0, -4.0, 8.0, 8.0));
    }

    /**
     * Returns the scale type that was set when the renderer was constructed.
     *
     * @return The scale type (never <code>null</code>).
     */
    public ScaleType getScaleType() {
        return this.scaleType;
    }
    
    /**
     * Returns the flag that controls whether or not outlines are drawn for 
     * the bubbles.  The default value is <code>false</code>.
     * 
     * @return A boolean. 
     */
    public boolean getDrawOutlines() {
        return this.drawOutlines;
    }
    
    /**
     * Sets the flag that controls whether or not outlines are drawn for the
     * bubbles and sends a change event to all registered listeners.
     * 
     * @param drawOutlines  the new flag value.
     */
    public void setDrawOutlines(boolean drawOutlines) {
        this.drawOutlines = drawOutlines;
        fireChangeEvent();
    }

    @Override
    public Range findDomainBounds(XYDataset dataset)  {
        XYZDataset datazet = (XYZDataset) dataset;
        double min = Double.POSITIVE_INFINITY;
        double max = Double.NEGATIVE_INFINITY;
        double factor = 1.0;
        if (this.scaleType == ScaleType.Y_AXIS) {
            // the factor will be the y-axis length / x-axis length
        }
        for (int s = 0; s < dataset.getSeriesCount(); s++) {
            if (this.isSeriesVisible(s)) {
                for (int i = 0; i < dataset.getItemCount(s); i++) {
                    double x = dataset.getXValue(s, i);
                    double z = datazet.getZValue(s, i);
                    double zAdj = Double.isNaN(z) ? 0.0 : z / 2.0;
                    double xLow = x - zAdj;
                    double xHigh = x + zAdj;
                    if (!Double.isNaN(x)) {
                        min = Math.min(min, xLow);
                        max = Math.max(max, xHigh);
                    }
                }
            }
        }
        return (min < max) ? new Range(min, max) : null;
    }

    @Override
    public Range findRangeBounds(XYDataset dataset) {
        return findRangeBounds(dataset, true);
    }

    @Override
    protected Range findRangeBounds(XYDataset dataset, 
            boolean includeInterval) {
        XYZDataset datazet = (XYZDataset) dataset;
        double min = Double.POSITIVE_INFINITY;
        double max = Double.NEGATIVE_INFINITY;
        for (int s = 0; s < dataset.getSeriesCount(); s++) {
            if (this.isSeriesVisible(s)) {
                for (int i = 0; i < dataset.getItemCount(s); i++) {
                    double y = dataset.getYValue(s, i);
                    double z = datazet.getZValue(s, i);
                    double zAdj = Double.isNaN(z) ? 0.0 : z / 2.0;
                    double yLow = y - zAdj;
                    double yHigh = y + zAdj;
                    if (!Double.isNaN(y)) {
                        min = Math.min(min, yLow);
                        max = Math.max(max, yHigh);
                    }
                }
            }
        }
        return (min < max) ? new Range(min, max) : null;
    }

    /**
     * Draws the visual representation of a single data item.
     *
     * @param g2  the graphics target (<code>null</code> not permitted).
     * @param state  the renderer state.
     * @param dataArea  the area within which the data is being drawn.
     * @param info  collects information about the drawing.
     * @param plot  the plot (can be used to obtain standard color
     *              information etc).
     * @param domainAxis  the domain (horizontal) axis.
     * @param rangeAxis  the range (vertical) axis.
     * @param dataset  the dataset (an {@link XYZDataset} is expected).
     * @param series  the series index (zero-based).
     * @param item  the item index (zero-based).
     * @param crosshairState  crosshair information for the plot
     *                        (<code>null</code> permitted).
     * @param pass  the pass index.
     */
    @Override
    public void drawItem(Graphics2D g2, XYItemRendererState state,
            Rectangle2D dataArea, PlotRenderingInfo info, XYPlot plot,
            ValueAxis domainAxis, ValueAxis rangeAxis, XYDataset dataset,
            int series, int item, CrosshairState crosshairState, int pass) {

        // return straight away if the item is not visible
        if (!getItemVisible(series, item)) {
            return;
        }

        PlotOrientation orientation = plot.getOrientation();
        double x = dataset.getXValue(series, item);
        double y = dataset.getYValue(series, item);
        double z = Double.NaN;
        if (dataset instanceof XYZDataset) {
            XYZDataset xyzData = (XYZDataset) dataset;
            z = xyzData.getZValue(series, item);
        }
        if (Double.isNaN(z)) {
            return;
        }
        RectangleEdge domainAxisLocation = plot.getDomainAxisEdge();
        RectangleEdge rangeAxisLocation = plot.getRangeAxisEdge();
        double transX = domainAxis.valueToJava2D(x, dataArea, 
                domainAxisLocation);
        double transY = rangeAxis.valueToJava2D(y, dataArea, rangeAxisLocation);

        double transDomain = 0.0;
        double transRange = 0.0;
        double zero;

        if (this.scaleType.equals(ScaleType.X_AXIS)) {
            zero = domainAxis.valueToJava2D(0.0, dataArea, domainAxisLocation);
            transDomain = domainAxis.valueToJava2D(z, dataArea,
                    domainAxisLocation) - zero;
            transRange = transDomain;   
        } else if (this.scaleType.equals(ScaleType.Y_AXIS)) {
            zero = rangeAxis.valueToJava2D(0.0, dataArea, rangeAxisLocation);
            transRange = zero - rangeAxis.valueToJava2D(z, dataArea,
                    rangeAxisLocation);
            transDomain = transRange;                
        } else if (this.scaleType.equals(ScaleType.BOTH_AXES)) {
            double xzero = domainAxis.valueToJava2D(0.0, dataArea,
                    domainAxisLocation);
            double yzero = rangeAxis.valueToJava2D(0.0, dataArea, 
                    rangeAxisLocation);
            transDomain = domainAxis.valueToJava2D(z, dataArea,
                    domainAxisLocation) - xzero;
            transRange = yzero - rangeAxis.valueToJava2D(z, dataArea,
                    rangeAxisLocation); 
        }
        transDomain = Math.abs(transDomain);
        transRange = Math.abs(transRange);
        Ellipse2D circle = null;
        if (orientation == PlotOrientation.VERTICAL) {
            circle = new Ellipse2D.Double(transX - transDomain / 2.0,
                    transY - transRange / 2.0, transDomain, transRange);
        } else if (orientation == PlotOrientation.HORIZONTAL) {
            circle = new Ellipse2D.Double(transY - transRange / 2.0,
                    transX - transDomain / 2.0, transRange, transDomain);
        } else {
            throw new IllegalStateException("Unrecognised plot orientation.");
        }
        Paint itemPaint = getItemPaint(series, item);
        if (itemPaint instanceof RadialGradientPaint) {
            RadialGradientPaint source = (RadialGradientPaint) itemPaint;
            Point2D center = new Point2D.Double(circle.getCenterX(), 
                    circle.getCenterY());
            RadialGradientPaint rgp = new RadialGradientPaint(center, 
                    (float) (Math.max(transRange, transDomain) / 2.0), 
                    source.getFractions(), source.getColors());
            g2.setPaint(rgp);
            g2.fill(circle);
        } else {
            g2.setPaint(itemPaint);
            g2.fill(circle);
        }
        if (this.drawOutlines) {
            g2.setStroke(getItemOutlineStroke(series, item));
            g2.setPaint(getItemOutlinePaint(series, item));
            g2.draw(circle);
        }
        if (isItemLabelVisible(series, item)) {
            if (orientation == PlotOrientation.VERTICAL) {
                drawItemLabel(g2, orientation, dataset, series, item,
                        transX, transY, false);
            } else if (orientation == PlotOrientation.HORIZONTAL) {
                drawItemLabel(g2, orientation, dataset, series, item,
                        transY, transX, false);
            }
        }

        // add an entity if this info is being collected
        EntityCollection entities;
        if (info != null) {
            entities = info.getOwner().getEntityCollection();
            if (entities != null && circle.intersects(dataArea)) {
                addEntity(entities, circle, dataset, series, item,
                        circle.getCenterX(), circle.getCenterY());
            }
        }

        int domainAxisIndex = plot.getDomainAxisIndex(domainAxis);
        int rangeAxisIndex = plot.getRangeAxisIndex(rangeAxis);
        updateCrosshairValues(crosshairState, x, y, domainAxisIndex,
                rangeAxisIndex, transX, transY, orientation);
    }

    /**
     * Returns a legend item for the specified series.  The default method
     * is overridden so that the legend displays circles for all series.
     *
     * @param datasetIndex  the dataset index (zero-based).
     * @param series  the series index (zero-based).
     *
     * @return A legend item for the series.
     */
    @Override
    public LegendItem getLegendItem(int datasetIndex, int series) {
        LegendItem result = null;
        XYPlot plot = getPlot();
        if (plot == null) {
            return null;
        }

        XYDataset dataset = plot.getDataset(datasetIndex);
        if (dataset != null) {
            if (getItemVisible(series, 0)) {
                String label = getLegendItemLabelGenerator().generateLabel(
                        dataset, series);
                String description = label;
                String toolTipText = null;
                if (getLegendItemToolTipGenerator() != null) {
                    toolTipText = getLegendItemToolTipGenerator().generateLabel(
                            dataset, series);
                }
                String urlText = null;
                if (getLegendItemURLGenerator() != null) {
                    urlText = getLegendItemURLGenerator().generateLabel(
                            dataset, series);
                }
                Shape shape = lookupLegendShape(series);
                Paint paint = lookupSeriesPaint(series);
                Paint outlinePaint = lookupSeriesOutlinePaint(series);
                Stroke outlineStroke = lookupSeriesOutlineStroke(series);
                result = new LegendItem(label, description, toolTipText,
                        urlText, shape, paint, outlineStroke, outlinePaint);
                result.setLabelFont(lookupLegendTextFont(series));
                Paint labelPaint = lookupLegendTextPaint(series);
                if (labelPaint != null) {
                    result.setLabelPaint(labelPaint);
                }
                result.setDataset(dataset);
                result.setDatasetIndex(datasetIndex);
                result.setSeriesKey(dataset.getSeriesKey(series));
                result.setSeriesIndex(series);
            }
        }
        return result;
    }

    /**
     * Tests this renderer for equality with an arbitrary object.
     *
     * @param obj  the object (<code>null</code> permitted).
     *
     * @return A boolean.
     */
    @Override
    public boolean equals(Object obj) {
        if (obj == this) {
            return true;
        }
        if (!(obj instanceof XYBubbleRenderer)) {
            return false;
        }
        XYBubbleRenderer that = (XYBubbleRenderer) obj;
        if (this.scaleType != that.scaleType) {
            return false;
        }
        return super.equals(obj);
    }

    /**
     * Returns a clone of the renderer.
     *
     * @return A clone.
     *
     * @throws CloneNotSupportedException  if the renderer cannot be cloned.
     */
    @Override
    public Object clone() throws CloneNotSupportedException {
        return super.clone();
    }

}
