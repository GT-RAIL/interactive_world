package edu.wpi.rail.jinteractiveworld.ros;

import edu.wpi.rail.jrosbridge.Ros;
import edu.wpi.rail.jrosbridge.Service;

public class InteractiveWorldLearner {

	public static final String NODE_NAME = "interactive_world_learner";

	public static final String LEARN_HYPOTHESES_SERVICE_NAME = "/" + InteractiveWorldLearner.NODE_NAME + "/learn_hypotheses";

	private Ros ros;

	public InteractiveWorldLearner() {
		ros = new Ros();
		System.out.print("Attempting connection to rosbridge...");
		if (!ros.connect()) {
			System.out.println("No connection made. Verify rosbridge_websocket is running.");
			System.exit(-1);
		}
		System.out.println(" success!");

		// setup the services
		Service s = new Service(ros, InteractiveWorldLearner.LEARN_HYPOTHESES_SERVICE_NAME, "std_srvs/Empty");
		s.advertiseService(null);
	}

	public void spin() throws InterruptedException {
		while (true) {
			Thread.sleep(1000);
		}
	}

	public static void main(String[] args) throws InterruptedException {
		// create the learner object
		InteractiveWorldLearner l = new InteractiveWorldLearner();
		l.spin();
	}
}
