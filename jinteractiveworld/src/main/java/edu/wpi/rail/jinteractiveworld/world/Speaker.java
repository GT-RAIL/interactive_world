package edu.wpi.rail.jinteractiveworld.world;

/**
 * The Speaker object in the world.
 * 
 * @author Russell Toris -- rctoris@wpi.edu
 * @version March 3, 2014
 */
public class Speaker extends Furniture {

	/**
	 * The name of the furniture.
	 */
	private static final String NAME = "Speaker";

	/**
	 * The width of the furniture.
	 */
	private static final double WIDTH = 0.19;

	/**
	 * The height of the furniture.
	 */
	private static final double HEIGHT = 0.19;

	/**
	 * Create a new Speaker object.
	 */
	public Speaker() {
		super(Speaker.NAME, Speaker.WIDTH, Speaker.HEIGHT);
	}
}
