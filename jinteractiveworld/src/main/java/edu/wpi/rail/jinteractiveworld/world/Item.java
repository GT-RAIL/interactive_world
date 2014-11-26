package edu.wpi.rail.jinteractiveworld.world;

/**
 * An Item is an abstract class that will be the parent class of any real world
 * item in the interactive world such as a spoon.
 * 
 * @author Russell Toris -- rctoris@wpi.edu
 * @version February 17, 2014
 */
public abstract class Item extends Object {

	/**
	 * Create a new Item with the given name.
	 * 
	 * @param name
	 *            The name of the Item.
	 * @param width
	 *            The width the Item.
	 * @param height
	 *            The height of the Item.
	 */
	public Item(String name, double width, double height) {
		super(name, width, height);
	}
}
