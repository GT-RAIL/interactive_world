package edu.wpi.rail.jinteractiveworld.world;

/**
 * The Counter object in the world.
 * 
 * @author Russell Toris -- rctoris@wpi.edu
 * @version March 3, 2014
 */
public class Counter extends Furniture {

	/**
	 * The name of the furniture.
	 */
	private static final String NAME = "Counter";

	/**
	 * The width of the furniture.
	 */
	private static final double WIDTH = 1.08;

	/**
	 * The height of the furniture.
	 */
	private static final double HEIGHT = 0.82;

	/**
	 * Create a new Counter object.
	 */
	public Counter() {
		super(Counter.NAME, Counter.WIDTH, Counter.HEIGHT);
	}
}
