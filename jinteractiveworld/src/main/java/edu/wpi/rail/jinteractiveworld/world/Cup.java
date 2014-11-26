package edu.wpi.rail.jinteractiveworld.world;

/**
 * The Cup object in the world.
 * 
 * @author Russell Toris -- rctoris@wpi.edu
 * @version February 17, 2014
 */
public class Cup extends Item {

	/**
	 * The name of the item.
	 */
	private static final String NAME = "Cup";

	/**
	 * The width of the item.
	 */
	private static final double WIDTH = 0.061;

	/**
	 * The height of the item.
	 */
	private static final double HEIGHT = 0.061;

	/**
	 * Create a new Cup object.
	 */
	public Cup() {
		super(Cup.NAME, Cup.WIDTH, Cup.HEIGHT);
	}
}
