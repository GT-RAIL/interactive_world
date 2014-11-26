package edu.wpi.rail.jinteractiveworld.ros.msgs.interactiveworldmsgs;

import static org.junit.Assert.*;

import javax.json.Json;
import javax.json.JsonObject;

import org.junit.Before;
import org.junit.Test;

import edu.wpi.rail.jinteractiveworld.model.transform.RotationMatrix;
import edu.wpi.rail.jinteractiveworld.model.transform.Vector3;
import edu.wpi.rail.jinteractiveworld.world.Bed;
import edu.wpi.rail.jrosbridge.messages.Message;
import edu.wpi.rail.jrosbridge.messages.geometry.Point;
import edu.wpi.rail.jrosbridge.messages.geometry.Pose;
import edu.wpi.rail.jrosbridge.messages.geometry.Quaternion;

public class TestPlacement {

	private Placement empty, p1, p2;

	@Before
	public void setUp() {
		empty = new Placement();
		p1 = new Placement(new Object("name", 1.2, 3.4), new Pose(new Point(1,
				2, 3), new Quaternion(4, 5, 6, 7)));
		p2 = new Placement(new edu.wpi.rail.jinteractiveworld.model.Placement(
				new Bed(), new RotationMatrix(Math.PI / 4.0,
						RotationMatrix.RotationType.Z_ROTATION), new Vector3(1,
						2, 3)));
	}

	@Test
	public void testConstructor() {
		assertEquals(new Object(), empty.getObj());
		assertEquals(new Pose(), empty.getPose());

		assertEquals(
				"{\"obj\":{\"name\":\"\",\"width\":0.0,\"height\":0.0},"
						+ "\"pose\":{\"position\":{\"x\":0.0,\"y\":0.0,\"z\":0.0},"
						+ "\"orientation\":{\"x\":0.0,\"y\":0.0,\"z\":0.0,\"w\":0.0}}}",
				empty.toString());

		assertEquals(2, empty.toJsonObject().size());
		assertEquals(
				new Object(),
				Object.fromJsonObject(empty.toJsonObject().getJsonObject(
						Placement.FIELD_OBJ)));
		assertEquals(
				new Pose(),
				Pose.fromJsonObject(empty.toJsonObject().getJsonObject(
						Placement.FIELD_POSE)));

		assertEquals(Placement.TYPE, empty.getMessageType());
	}

	@Test
	public void testObjectAndPoseConstructor() {
		assertEquals(new Object("name", 1.2, 3.4), p1.getObj());
		assertEquals(new Pose(new Point(1, 2, 3), new Quaternion(4, 5, 6, 7)),
				p1.getPose());

		assertEquals(
				"{\"obj\":{\"name\":\"name\",\"width\":1.2,\"height\":3.4},"
						+ "\"pose\":{\"position\":{\"x\":1.0,\"y\":2.0,\"z\":3.0},"
						+ "\"orientation\":{\"x\":4.0,\"y\":5.0,\"z\":6.0,\"w\":7.0}}}",
				p1.toString());

		assertEquals(2, p1.toJsonObject().size());
		assertEquals(
				new Object("name", 1.2, 3.4),
				Object.fromJsonObject(p1.toJsonObject().getJsonObject(
						Placement.FIELD_OBJ)));
		assertEquals(
				new Pose(new Point(1, 2, 3), new Quaternion(4, 5, 6, 7)),
				Pose.fromJsonObject(p1.toJsonObject().getJsonObject(
						Placement.FIELD_POSE)));

		assertEquals(Placement.TYPE, p1.getMessageType());
	}

	@Test
	public void testPlacementConstructor() {
		Bed b = new Bed();
		assertEquals(new Object(b.getName(), b.getWidth(), b.getHeight()),
				p2.getObj());
		assertEquals(new Pose(new Point(1, 2, 3), new Quaternion(0, 0,
				0.3826834323650898, 0.9238795325112867)), p2.getPose());

		assertEquals(
				"{\"obj\":{\"name\":\"Bed\",\"width\":1.7,\"height\":2.15},"
						+ "\"pose\":{\"position\":{\"x\":1.0,\"y\":2.0,\"z\":3.0},"
						+ "\"orientation\":{\"x\":0.0,\"y\":0.0,\"z\":0.3826834323650898,\"w\":0.9238795325112867}}}",
				p2.toString());

		assertEquals(2, p2.toJsonObject().size());
		assertEquals(
				new Object(b.getName(), b.getWidth(), b.getHeight()),
				Object.fromJsonObject(p2.toJsonObject().getJsonObject(
						Placement.FIELD_OBJ)));
		assertEquals(new Pose(new Point(1, 2, 3), new Quaternion(0, 0,
				0.3826834323650898, 0.9238795325112867)),
				Pose.fromJsonObject(p2.toJsonObject().getJsonObject(
						Placement.FIELD_POSE)));

		assertEquals(Placement.TYPE, p2.getMessageType());
	}

	@Test
	public void testSetMessageType() {
		empty.setMessageType("test");
		assertEquals("test", empty.getMessageType());
	}

	@Test
	public void testHashCode() {
		assertEquals(empty.toString().hashCode(), empty.hashCode());
		assertEquals(p1.toString().hashCode(), p1.hashCode());
		assertEquals(p2.toString().hashCode(), p2.hashCode());
	}

	@Test
	public void testEquals() {
		assertFalse(empty.equals(p1));
		assertFalse(empty.equals(p2));
		assertFalse(p1.equals(empty));
		assertFalse(p1.equals(p2));
		assertFalse(p2.equals(p1));
		assertFalse(p2.equals(empty));

		assertTrue(p1.equals(p1));
		assertTrue(p2.equals(p2));
		assertTrue(empty.equals(empty));
	}

	@Test
	public void testEqualsWrongPlacement() {
		assertFalse(p1.equals(new String(p1.toString())));
	}

	@Test
	public void testClone() {
		Placement clone = p1.clone();
		assertEquals(p1.toString(), clone.toString());
		assertEquals(p1.toJsonObject(), clone.toJsonObject());
		assertEquals(p1.getMessageType(), clone.getMessageType());
		assertEquals(p1.getObj(), clone.getObj());
		assertEquals(p1.getPose(), clone.getPose());
		assertNotSame(p1, clone);
		assertNotSame(p1.toString(), clone.toString());
		assertNotSame(p1.toJsonObject(), clone.toJsonObject());
	}

	@Test
	public void testFromJsonString() {
		Placement p = Placement.fromJsonString(p1.toString());
		assertEquals(p1.toString(), p.toString());
		assertEquals(p1.toJsonObject(), p.toJsonObject());
		assertEquals(p1.getMessageType(), p.getMessageType());
		assertEquals(p1.getObj(), p.getObj());
		assertEquals(p1.getPose(), p.getPose());
		assertNotSame(p1, p);
		assertNotSame(p1.toString(), p.toString());
		assertNotSame(p1.toJsonObject(), p.toJsonObject());
	}

	@Test
	public void testFromMessage() {
		Message m = new Message(p1.toString());
		Placement p = Placement.fromMessage(m);
		assertEquals(p1.toString(), p.toString());
		assertEquals(p1.toJsonObject(), p.toJsonObject());
		assertEquals(p1.getMessageType(), p.getMessageType());
		assertEquals(p1.getObj(), p.getObj());
		assertEquals(p1.getPose(), p.getPose());
		assertNotSame(p1, p);
		assertNotSame(p1.toString(), p.toString());
		assertNotSame(p1.toJsonObject(), p.toJsonObject());
	}

	@Test
	public void testFromJsonObject() {
		JsonObject jsonObject = Json.createObjectBuilder()
				.add(Placement.FIELD_OBJ, p1.getObj().toJsonObject())
				.add(Placement.FIELD_POSE, p1.getPose().toJsonObject()).build();
		Placement p = Placement.fromJsonObject(jsonObject);
		assertEquals(p1.toString(), p.toString());
		assertEquals(p1.toJsonObject(), p.toJsonObject());
		assertEquals(p1.getMessageType(), p.getMessageType());
		assertEquals(p1.getObj(), p.getObj());
		assertEquals(p1.getPose(), p.getPose());
		assertNotSame(p1, p);
		assertNotSame(p1.toString(), p.toString());
		assertNotSame(p1.toJsonObject(), p.toJsonObject());
	}

	@Test
	public void testFromJsonObjectNoObj() {
		JsonObject jsonObject = Json.createObjectBuilder()
				.add(Placement.FIELD_POSE, p1.getPose().toJsonObject()).build();
		Placement p = Placement.fromJsonObject(jsonObject);
		assertEquals(new Object(), p.getObj());
		assertEquals(p1.getPose(), p.getPose());
	}

	@Test
	public void testFromJsonObjectNoPose() {
		JsonObject jsonObject = Json.createObjectBuilder()
				.add(Placement.FIELD_OBJ, p1.getObj().toJsonObject()).build();
		Placement p = Placement.fromJsonObject(jsonObject);
		assertEquals(p1.getObj(), p.getObj());
		assertEquals(new Pose(), p.getPose());
	}
}
