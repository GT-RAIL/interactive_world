package edu.wpi.rail.jinteractiveworld.visualization;

import static org.junit.Assert.*;

import org.junit.Test;

import edu.wpi.rail.jinteractiveworld.model.State;
import edu.wpi.rail.jinteractiveworld.model.transform.RotationMatrix;
import edu.wpi.rail.jinteractiveworld.model.transform.Transform;
import edu.wpi.rail.jinteractiveworld.model.transform.Vector3;
import edu.wpi.rail.jinteractiveworld.visualization.model.StateFrame;
import edu.wpi.rail.jinteractiveworld.world.Bed;
import edu.wpi.rail.jinteractiveworld.world.Burner;
import edu.wpi.rail.jinteractiveworld.world.Cabinet;
import edu.wpi.rail.jinteractiveworld.world.Chair;
import edu.wpi.rail.jinteractiveworld.world.CoffeeTable;
import edu.wpi.rail.jinteractiveworld.world.Couch;
import edu.wpi.rail.jinteractiveworld.world.Counter;
import edu.wpi.rail.jinteractiveworld.world.Cup;
import edu.wpi.rail.jinteractiveworld.world.DiningTable;
import edu.wpi.rail.jinteractiveworld.world.Dresser;
import edu.wpi.rail.jinteractiveworld.world.Fork;
import edu.wpi.rail.jinteractiveworld.world.Magazines;
import edu.wpi.rail.jinteractiveworld.world.Nightstand;
import edu.wpi.rail.jinteractiveworld.world.Oven;
import edu.wpi.rail.jinteractiveworld.world.Pillow;
import edu.wpi.rail.jinteractiveworld.world.Plate;
import edu.wpi.rail.jinteractiveworld.world.Refrigerator;
import edu.wpi.rail.jinteractiveworld.world.Sink;
import edu.wpi.rail.jinteractiveworld.world.SinkUnit;
import edu.wpi.rail.jinteractiveworld.world.Speaker;
import edu.wpi.rail.jinteractiveworld.world.Spoon;
import edu.wpi.rail.jinteractiveworld.world.TV;

public class TestStateFrame {

	@Test
	public void testConstructor() {
		StateFrame sf = new StateFrame();
		sf.setVisible(true);
		assertTrue(sf.isVisible());
	}

	@Test
	public void testStateConstructor() {
		State s = new State();
		s.add(new Bed(), new Transform());
		s.add(new Burner(), new Transform());
		s.add(new Cabinet(), new Transform());
		s.add(new CoffeeTable(), new Transform());
		s.add(new Couch(), new Transform());
		s.add(new Counter(), new Transform());
		s.add(new Cup(), new Transform());
		s.add(new DiningTable(), new Transform());
		s.add(new Dresser(), new Transform());
		s.add(new Fork(), new Transform());
		s.add(new Magazines(), new Transform());
		s.add(new Nightstand(), new Transform());
		s.add(new Oven(), new Transform());
		s.add(new Pillow(), new Transform());
		s.add(new Plate(), new Transform());
		s.add(new Refrigerator(), new Transform());
		s.add(new Sink(), new Transform());
		s.add(new SinkUnit(), new Transform());
		s.add(new Speaker(), new Transform());
		s.add(new Spoon(), new Transform());
		s.add(new TV(), new Transform());
		s.add(new Plate(), new RotationMatrix(Math.PI,
				RotationMatrix.RotationType.Z_ROTATION), new Vector3(0.125,
				0.25));
		s.add(new Chair(), new RotationMatrix(Math.PI / 3.0,
				RotationMatrix.RotationType.Z_ROTATION), new Vector3(-0.15,
				0.15));
		s.add(new Cup(), new RotationMatrix(), new Vector3(-0.25, -0.15));
		s.add(new Fork(), new RotationMatrix(-Math.PI / 2.5,
				RotationMatrix.RotationType.Z_ROTATION), new Vector3(0.111,
				-0.315));
		s.add(new Spoon(), new RotationMatrix(-Math.PI / 2,
				RotationMatrix.RotationType.Z_ROTATION), new Vector3(-0.111,
				-0.315));

		StateFrame sf = new StateFrame(s);
		sf.setVisible(true);
		assertTrue(sf.isVisible());
	}
}
