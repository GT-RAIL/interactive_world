package edu.wpi.rail.jinteractiveworld.world;

/**
 * The Sink unit object in the world.
 * 
 * @author Russell Toris -- rctoris@wpi.edu
 * @version March 3, 2014
 */
public class SinkUnit extends Furniture {

	/**
	 * The name of the furniture.
	 */
	private static final String NAME = "Sink Unit";

	/**
	 * The width of the furniture.
	 */
	private static final double WIDTH = 2.91;

	/**
	 * The height of the furniture.
	 */
	private static final double HEIGHT = 0.775;

	/**
	 * Create a new SinkUnit object.
	 */
	public SinkUnit() {
		super(SinkUnit.NAME, SinkUnit.WIDTH, SinkUnit.HEIGHT);
	}
}
