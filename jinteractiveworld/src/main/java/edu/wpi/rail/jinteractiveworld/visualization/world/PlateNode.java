package edu.wpi.rail.jinteractiveworld.visualization.world;

import java.awt.Color;

import org.piccolo2d.PNode;
import org.piccolo2d.nodes.PPath;
import org.piccolo2d.nodes.PText;

import edu.wpi.rail.jinteractiveworld.visualization.model.StateFrame;
import edu.wpi.rail.jinteractiveworld.world.Plate;

/**
 * The plate visualization node.
 * 
 * @author Russell Toris -- rctoris@wpi.edu
 * @version March 3, 2014
 */
@SuppressWarnings("serial")
public class PlateNode extends PNode {

	/**
	 * The default color for the object in the visualization.
	 */
	private static final Color COLOR = new Color((int) (Math.random() * 255),
			(int) (Math.random() * 255), (int) (Math.random() * 255));

	/**
	 * Create a new plate node for the visualization.
	 */
	public PlateNode() {
		super();

		// get the size from the Object
		Plate p = new Plate();
		double w = p.getWidth() * StateFrame.PIXELS_PER_METER;
		double h = p.getHeight() * StateFrame.PIXELS_PER_METER;

		// create the shape
		PPath shape = PPath.createEllipse(-w / 2.0, -h / 2.0, w, h);
		// set the color
		shape.setPaint(COLOR);
		// add the shape
		this.addChild(shape);

		// create the label
		PText text = new PText(p.getName());
		text.setOffset(-text.getWidth() / 2.0, -text.getHeight() / 2.0);
		// add the label
		this.addChild(text);
	}
}
