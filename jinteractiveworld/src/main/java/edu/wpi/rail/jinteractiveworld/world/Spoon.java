package edu.wpi.rail.jinteractiveworld.world;

/**
 * The Spoon object in the world.
 * 
 * @author Russell Toris -- rctoris@wpi.edu
 * @version February 17, 2014
 */
public class Spoon extends Item {

	/**
	 * The name of the item.
	 */
	private static final String NAME = "Spoon";

	/**
	 * The width of the item.
	 */
	private static final double WIDTH = 0.032;

	/**
	 * The height of the item.
	 */
	private static final double HEIGHT = 0.15;

	/**
	 * Create a new Spoon object.
	 */
	public Spoon() {
		super(Spoon.NAME, Spoon.WIDTH, Spoon.HEIGHT);
	}
}
