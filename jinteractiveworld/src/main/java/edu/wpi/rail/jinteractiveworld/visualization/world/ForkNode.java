package edu.wpi.rail.jinteractiveworld.visualization.world;

import java.awt.Color;

import org.piccolo2d.PNode;
import org.piccolo2d.nodes.PPath;
import org.piccolo2d.nodes.PText;

import edu.wpi.rail.jinteractiveworld.visualization.model.StateFrame;
import edu.wpi.rail.jinteractiveworld.world.Fork;

/**
 * The fork visualization node.
 * 
 * @author Russell Toris -- rctoris@wpi.edu
 * @version March 3, 2014
 */
@SuppressWarnings("serial")
public class ForkNode extends PNode {

	/**
	 * The default color for the object in the visualization.
	 */
	private static final Color COLOR = new Color((int) (Math.random() * 255),
			(int) (Math.random() * 255), (int) (Math.random() * 255));

	/**
	 * Create a new fork node for the visualization.
	 */
	public ForkNode() {
		super();

		// get the size from the Item
		Fork f = new Fork();
		double w = f.getWidth() * StateFrame.PIXELS_PER_METER;
		double h = f.getHeight() * StateFrame.PIXELS_PER_METER;
		int r = 10;

		// create the shape
		PPath shape = PPath
				.createRoundRectangle(-w / 2.0, -h / 2.0, w, h, r, r);
		// set the color
		shape.setPaint(COLOR);
		// add the shape
		this.addChild(shape);

		// create the label
		PText text = new PText(f.getName());
		text.setRotation(Math.PI / 2.0);
		text.setOffset(text.getHeight() / 2.0, -text.getWidth() / 2.0);
		// add the label
		this.addChild(text);
	}
}
