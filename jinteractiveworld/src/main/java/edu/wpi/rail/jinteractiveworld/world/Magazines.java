package edu.wpi.rail.jinteractiveworld.world;

/**
 * The Magazines object in the world.
 * 
 * @author Russell Toris -- rctoris@wpi.edu
 * @version March 3, 2014
 */
public class Magazines extends Item {

	/**
	 * The name of the item.
	 */
	private static final String NAME = "Magazines";

	/**
	 * The width of the item.
	 */
	private static final double WIDTH = 0.467;

	/**
	 * The height of the item.
	 */
	private static final double HEIGHT = 0.399;

	/**
	 * Create a new Magazines object.
	 */
	public Magazines() {
		super(Magazines.NAME, Magazines.WIDTH, Magazines.HEIGHT);
	}
}
