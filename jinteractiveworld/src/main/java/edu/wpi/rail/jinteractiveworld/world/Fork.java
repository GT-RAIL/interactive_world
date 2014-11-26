package edu.wpi.rail.jinteractiveworld.world;

/**
 * The Fork object in the world.
 * 
 * @author Russell Toris -- rctoris@wpi.edu
 * @version February 17, 2014
 */
public class Fork extends Item {

	/**
	 * The name of the item.
	 */
	private static final String NAME = "Fork";

	/**
	 * The width of the item.
	 */
	private static final double WIDTH = 0.025;

	/**
	 * The height of the item.
	 */
	private static final double HEIGHT = 0.195;

	/**
	 * Create a new Fork object.
	 */
	public Fork() {
		super(Fork.NAME, Fork.WIDTH, Fork.HEIGHT);
	}
}
