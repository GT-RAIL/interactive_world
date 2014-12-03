package edu.wpi.rail.jinteractiveworld.ros;

import edu.wpi.rail.jinteractiveworld.ros.srvs.interactiveworldmsgs.LearnModels;
import edu.wpi.rail.jrosbridge.Ros;
import edu.wpi.rail.jrosbridge.Service;
import edu.wpi.rail.jrosbridge.callback.CallServiceCallback;
import edu.wpi.rail.jrosbridge.services.ServiceRequest;

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
		Service s = new Service(ros, InteractiveWorldLearner.LEARN_HYPOTHESES_SERVICE_NAME, "interactive_world_msgs/LearnModels");
		s.advertiseService(new LearnModelsCallback());
	}

	public static void spin() {
		while (true) {
			try {
				Thread.sleep(1000);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}

	private class LearnModelsCallback implements CallServiceCallback {

		@Override
		public void handleServiceCall(ServiceRequest request) {
			System.out.println("Got request...");
			LearnModels.Request req = LearnModels.Request.fromJsonObject(request.toJsonObject());
			System.out.println("Parsed request...");
			System.out.println(req);
			System.out.println(req.toJsonObject().equals(request.toJsonObject()));
		}
	}

	public static void main(String[] args) {
		// create the learner object
		InteractiveWorldLearner l = new InteractiveWorldLearner();
		InteractiveWorldLearner.spin();
	}
}
