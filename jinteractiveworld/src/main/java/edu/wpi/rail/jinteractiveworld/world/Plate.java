package edu.wpi.rail.jinteractiveworld.world;

/**
 * The Plate object in the world.
 * 
 * @author Russell Toris -- rctoris@wpi.edu
 * @version February 17, 2014
 */
public class Plate extends Item {

	/**
	 * The name of the item.
	 */
	private static final String NAME = "Plate";

	/**
	 * The width of the item.
	 */
	private static final double WIDTH = 0.149;

	/**
	 * The height of the item.
	 */
	private static final double HEIGHT = 0.149;

	/**
	 * Create a new Plate object.
	 */
	public Plate() {
		super(Plate.NAME, Plate.WIDTH, Plate.HEIGHT);
	}
}
