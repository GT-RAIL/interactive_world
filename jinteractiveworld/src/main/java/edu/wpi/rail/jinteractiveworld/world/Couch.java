package edu.wpi.rail.jinteractiveworld.world;

/**
 * The Couch object in the world.
 * 
 * @author Russell Toris -- rctoris@wpi.edu
 * @version March 3, 2014
 */
public class Couch extends Furniture {

	/**
	 * The name of the furniture.
	 */
	private static final String NAME = "Couch";

	/**
	 * The width of the furniture.
	 */
	private static final double WIDTH = 2.99;

	/**
	 * The height of the furniture.
	 */
	private static final double HEIGHT = 1.68;

	/**
	 * Create a new Couch object.
	 */
	public Couch() {
		super(Couch.NAME, Couch.WIDTH, Couch.HEIGHT);
	}
}
