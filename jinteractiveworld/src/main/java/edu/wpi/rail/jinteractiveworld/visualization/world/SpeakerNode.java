package edu.wpi.rail.jinteractiveworld.visualization.world;

import java.awt.Color;

import org.piccolo2d.PNode;
import org.piccolo2d.nodes.PPath;
import org.piccolo2d.nodes.PText;

import edu.wpi.rail.jinteractiveworld.visualization.model.StateFrame;
import edu.wpi.rail.jinteractiveworld.world.Speaker;

/**
 * The Speaker visualization node.
 * 
 * @author Russell Toris -- rctoris@wpi.edu
 * @version March 3, 2014
 */
@SuppressWarnings("serial")
public class SpeakerNode extends PNode {

	/**
	 * The default color for the object in the visualization.
	 */
	private static final Color COLOR = new Color((int) (Math.random() * 255),
			(int) (Math.random() * 255), (int) (Math.random() * 255));

	/**
	 * Create a new Speaker node for the visualization.
	 */
	public SpeakerNode() {
		super();

		// get the size from the Object
		Speaker s = new Speaker();
		double w = s.getWidth() * StateFrame.PIXELS_PER_METER;
		double h = s.getHeight() * StateFrame.PIXELS_PER_METER;

		// create the shape
		PPath shape = PPath.createRectangle(-w / 2.0, -h / 2.0, w, h);
		// set the color
		shape.setPaint(COLOR);
		// add the shape
		this.addChild(shape);

		// create the label
		PText text = new PText(s.getName());
		text.setOffset(-text.getWidth() / 2.0, -text.getHeight() / 2.0);
		// add the label
		this.addChild(text);
	}
}
