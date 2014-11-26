package edu.wpi.rail.jinteractiveworld.visualization.model;

import javax.swing.JFrame;
import javax.swing.WindowConstants;

import org.piccolo2d.PCanvas;
import org.piccolo2d.PLayer;
import org.piccolo2d.PNode;
import org.piccolo2d.nodes.PPath;

import edu.wpi.rail.jinteractiveworld.model.Placement;
import edu.wpi.rail.jinteractiveworld.model.State;
import edu.wpi.rail.jinteractiveworld.model.transform.RotationMatrix;
import edu.wpi.rail.jinteractiveworld.model.transform.Transform;
import edu.wpi.rail.jinteractiveworld.model.transform.Vector3;
import edu.wpi.rail.jinteractiveworld.visualization.world.BedNode;
import edu.wpi.rail.jinteractiveworld.visualization.world.BurnerNode;
import edu.wpi.rail.jinteractiveworld.visualization.world.CabinetNode;
import edu.wpi.rail.jinteractiveworld.visualization.world.ChairNode;
import edu.wpi.rail.jinteractiveworld.visualization.world.CoffeeTableNode;
import edu.wpi.rail.jinteractiveworld.visualization.world.CouchNode;
import edu.wpi.rail.jinteractiveworld.visualization.world.CounterNode;
import edu.wpi.rail.jinteractiveworld.visualization.world.CupNode;
import edu.wpi.rail.jinteractiveworld.visualization.world.DiningTableNode;
import edu.wpi.rail.jinteractiveworld.visualization.world.DresserNode;
import edu.wpi.rail.jinteractiveworld.visualization.world.ForkNode;
import edu.wpi.rail.jinteractiveworld.visualization.world.MagazinesNode;
import edu.wpi.rail.jinteractiveworld.visualization.world.NightstandNode;
import edu.wpi.rail.jinteractiveworld.visualization.world.OvenNode;
import edu.wpi.rail.jinteractiveworld.visualization.world.PillowNode;
import edu.wpi.rail.jinteractiveworld.visualization.world.PlateNode;
import edu.wpi.rail.jinteractiveworld.visualization.world.RefrigeratorNode;
import edu.wpi.rail.jinteractiveworld.visualization.world.SinkNode;
import edu.wpi.rail.jinteractiveworld.visualization.world.SinkUnitNode;
import edu.wpi.rail.jinteractiveworld.visualization.world.SpeakerNode;
import edu.wpi.rail.jinteractiveworld.visualization.world.SpoonNode;
import edu.wpi.rail.jinteractiveworld.visualization.world.TVNode;
import edu.wpi.rail.jinteractiveworld.world.Bed;
import edu.wpi.rail.jinteractiveworld.world.Burner;
import edu.wpi.rail.jinteractiveworld.world.Cabinet;
import edu.wpi.rail.jinteractiveworld.world.Chair;
import edu.wpi.rail.jinteractiveworld.world.CoffeeTable;
import edu.wpi.rail.jinteractiveworld.world.Couch;
import edu.wpi.rail.jinteractiveworld.world.Counter;
import edu.wpi.rail.jinteractiveworld.world.Cup;
import edu.wpi.rail.jinteractiveworld.world.DiningTable;
import edu.wpi.rail.jinteractiveworld.world.Dresser;
import edu.wpi.rail.jinteractiveworld.world.Fork;
import edu.wpi.rail.jinteractiveworld.world.Magazines;
import edu.wpi.rail.jinteractiveworld.world.Nightstand;
import edu.wpi.rail.jinteractiveworld.world.Object;
import edu.wpi.rail.jinteractiveworld.world.Oven;
import edu.wpi.rail.jinteractiveworld.world.Pillow;
import edu.wpi.rail.jinteractiveworld.world.Plate;
import edu.wpi.rail.jinteractiveworld.world.Refrigerator;
import edu.wpi.rail.jinteractiveworld.world.Sink;
import edu.wpi.rail.jinteractiveworld.world.SinkUnit;
import edu.wpi.rail.jinteractiveworld.world.Speaker;
import edu.wpi.rail.jinteractiveworld.world.Spoon;

/**
 * The StateFrame visualizes a given state (i.e., a set of placements).
 * 
 * @author Russell Toris -- rctoris@wpi.edu
 * @version February 17, 2014
 */
@SuppressWarnings("serial")
public class StateFrame extends JFrame {

	/**
	 * The default width of the window.
	 */
	public static final int DEFAULT_WIDTH = 800;

	/**
	 * The default height of the window.
	 */
	public static final int DEFAULT_HEIGHT = 600;

	/**
	 * The default resolution of the viewer.
	 */
	public static final int PIXELS_PER_METER = 500;

	/**
	 * Create a new visualization with an empty state.
	 */
	public StateFrame() {
		this(new State());
	}

	/**
	 * Create a new visualization with the given state.
	 * 
	 * @param s
	 *            The state to visualize.
	 */
	public StateFrame(State s) {
		super();

		// create the scene
		PCanvas canvas = new PCanvas();

		// change the 0,0 to the center
		PLayer layer = canvas.getLayer();
		layer.setOffset(StateFrame.DEFAULT_WIDTH / 2.0,
				StateFrame.DEFAULT_HEIGHT / 2.0);

		// create the axis
		PPath xAxis = PPath.createLine(-StateFrame.DEFAULT_WIDTH * 10, 0,
				StateFrame.DEFAULT_WIDTH * 10, 0);
		PPath yAxis = PPath.createLine(0, -StateFrame.DEFAULT_HEIGHT * 10, 0,
				StateFrame.DEFAULT_HEIGHT * 10);
		layer.addChild(xAxis);
		layer.addChild(yAxis);

		// add each object to the scene
		for (Placement p : s.getPlacements()) {
			Object o = p.getObject();
			Transform tf = p.getTransform();

			// check the object type
			PNode node;
			if (o instanceof Bed) {
				node = new BedNode();
			} else if (o instanceof Burner) {
				node = new BurnerNode();
			} else if (o instanceof Cabinet) {
				node = new CabinetNode();
			} else if (o instanceof Chair) {
				node = new ChairNode();
			} else if (o instanceof CoffeeTable) {
				node = new CoffeeTableNode();
			} else if (o instanceof Couch) {
				node = new CouchNode();
			} else if (o instanceof Counter) {
				node = new CounterNode();
			} else if (o instanceof Cup) {
				node = new CupNode();
			} else if (o instanceof DiningTable) {
				node = new DiningTableNode();
			} else if (o instanceof Dresser) {
				node = new DresserNode();
			} else if (o instanceof Fork) {
				node = new ForkNode();
			} else if (o instanceof Magazines) {
				node = new MagazinesNode();
			} else if (o instanceof Nightstand) {
				node = new NightstandNode();
			} else if (o instanceof Oven) {
				node = new OvenNode();
			} else if (o instanceof Pillow) {
				node = new PillowNode();
			} else if (o instanceof Plate) {
				node = new PlateNode();
			} else if (o instanceof Refrigerator) {
				node = new RefrigeratorNode();
			} else if (o instanceof Sink) {
				node = new SinkNode();
			} else if (o instanceof SinkUnit) {
				node = new SinkUnitNode();
			} else if (o instanceof Speaker) {
				node = new SpeakerNode();
			} else if (o instanceof Spoon) {
				node = new SpoonNode();
			} else {
				node = new TVNode();
			}

			// create the offset
			Vector3 offset = tf.getVector3();
			double x = offset.getX() * StateFrame.PIXELS_PER_METER;
			double y = -offset.getY() * StateFrame.PIXELS_PER_METER;

			// create the rotation
			RotationMatrix r = tf.getRotationMatrix();
			double theta = r
					.getRotationAboutAxis(RotationMatrix.RotationType.Z_ROTATION);

			// add the object
			node.setRotation(theta);
			node.setOffset(x, y);
			layer.addChild(node);
		}

		// add the canvas and finish setting up the frame
		this.add(canvas);
		this.setSize(StateFrame.DEFAULT_WIDTH, StateFrame.DEFAULT_HEIGHT);
		this.setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);
	}
}
