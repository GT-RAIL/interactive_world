package edu.wpi.rail.jinteractiveworld.ros.srvs.interactiveworldmsgs;

import javax.json.Json;
import javax.json.JsonObject;

import edu.wpi.rail.jinteractiveworld.ros.msgs.interactiveworldmsgs.TaskModels;
import edu.wpi.rail.jinteractiveworld.ros.msgs.interactiveworldmsgs.TaskTrainingData;
import edu.wpi.rail.jrosbridge.services.ServiceRequest;
import edu.wpi.rail.jrosbridge.services.ServiceResponse;

import java.io.StringReader;
import java.util.Arrays;

/**
 * The interactive_world_msgs/LearnModels service.
 *
 * @author Russell Toris -- rctoris@wpi.edu
 * @version December 3, 2014
 */
public class LearnModels {

    /**
     * The service type.
     */
    public static final String TYPE = "interactive_world_msgs/LearnModels";

    /**
     * The service request for the interactive_world_msgs/LearnModels service.
     *
     * @author Russell Toris -- rctoris@wpi.edu
     * @version December 3, 2014
     */
    public static class Request extends ServiceRequest {

        /**
         * The name of the host field for the request.
         */
        public static final String FIELD_HOST = "host";

        /**
         * The name of the port field for the request.
         */
        public static final String FIELD_PORT = "port";

        /**
         * The name of the user field for the request.
         */
        public static final String FIELD_USER = "user";

        /**
         * The name of the password field for the request.
         */
        public static final String FIELD_PASSWORD = "password";

        /**
         * The name of the database field for the request.
         */
        public static final String FIELD_DATABASE = "database";

        /**
         * The name of the condition id field for the request.
         */
        public static final String FIELD_CONDITION_ID = "condition_id";

        /**
         * The name of the data field for the request.
         */
        public static final String FIELD_DATA = "data";

        private final TaskTrainingData data;
        private final int port, conditionId;
        private final String host, user, password, database;

        /**
         * Create a new empty LearnModels ServiceRequest.
         */
        public Request() {
            this("", 0, "", "", "", 0, new TaskTrainingData());
        }

        /**
         * Create a new LearnModels ServiceRequest.
         *
         * @param host
         *            The host of the request.
         * @param port
         *            The port of the request.
         * @param user
         *            The user of the request.
         * @param password
         *            The password of the request.
         * @param database
         *            The database of the request.
         * @param conditionId
         *            The condition ID of the request.
         * @param data
         *            The data of the request.
         */
        public Request(String host, int port, String user, String password, String database, int conditionId, TaskTrainingData data) {
            super(Json.createObjectBuilder().add(Request.FIELD_DATA, data.toJsonObject())
                    .add(Request.FIELD_CONDITION_ID, conditionId).build(), LearnModels.TYPE);
            this.host = host;
            this.port = port;
            this.user = user;
            this.password = password;
            this.database = database;
            this.conditionId = conditionId;
            this.data = data;
        }

        public int getPort() {
            return port;
        }

        public String getHost() {
            return host;
        }

        public String getUser() {
            return user;
        }

        public String getPassword() {
            return password;
        }

        public String getDatabase() {
            return database;
        }

        public int getConditionId() {
            return this.conditionId;
        }

        public TaskTrainingData getData() {
            return this.data;
        }

        /**
         * Create a clone of this LearnModels ServiceRequest.
         */
        @Override
        public Request clone() {
            return new LearnModels.Request(this.host, this.port, this.user, this.password, this.database, this.conditionId, this.data);
        }

        /**
         * Create a new LearnModels ServiceRequest based on the given JSON string. Any
         * missing values will be set to their defaults.
         *
         * @param jsonString
         *            The JSON string to parse.
         * @return A LearnModels ServiceRequest based on the given JSON string.
         */
        public static Request fromJsonString(String jsonString) {
            // convert to a ServiceRequest
            return LearnModels.Request.fromServiceRequest(new ServiceRequest(
                    jsonString));
        }

        /**
         * Create a new LearnModels ServiceRequest based on the given ServiceRequest.
         * Any missing values will be set to their defaults.
         *
         * @param req
         *            The ServiceRequest to parse.
         * @return A LearnModels ServiceRequest based on the given Message.
         */
        public static Request fromServiceRequest(ServiceRequest req) {
            // get it from the JSON object
            return LearnModels.Request.fromJsonObject(req.toJsonObject());
        }

        /**
         * Create a new LearnModels ServiceRequest based on the given JSON object. Any
         * missing values will be set to their defaults.
         *
         * @param jsonObject
         *            The JSON object to parse.
         * @return A LearnModels ServiceRequest based on the given JSON object.
         */
        public static Request fromJsonObject(JsonObject jsonObject) {
            String host = jsonObject.containsKey(Request.FIELD_HOST) ?
                    jsonObject.getString(Request.FIELD_HOST) : "";
            int port = jsonObject.containsKey(Request.FIELD_PORT) ? jsonObject
                    .getJsonNumber(Request.FIELD_PORT).intValue() : 0;
            String user = jsonObject.containsKey(Request.FIELD_USER) ?
                    jsonObject.getString(Request.FIELD_USER) : "";
            String password = jsonObject.containsKey(Request.FIELD_PASSWORD) ?
                    jsonObject.getString(Request.FIELD_PASSWORD) : "";
            String database = jsonObject.containsKey(Request.FIELD_DATABASE) ?
                    jsonObject.getString(Request.FIELD_DATABASE) : "";
            int conditionId = jsonObject.containsKey(Request.FIELD_CONDITION_ID) ? jsonObject
                    .getJsonNumber(Request.FIELD_CONDITION_ID).intValue() : 0;
            TaskTrainingData data = jsonObject.containsKey(Request.FIELD_DATA) ? TaskTrainingData
                    .fromJsonObject(jsonObject.getJsonObject(Request.FIELD_DATA))
                    : new TaskTrainingData();
            return new LearnModels.Request(host, port, user, password, database, conditionId, data);
        }
    }

    /**
     * The service response for the interactive_world_msgs/LearnModels service.
     *
     * @author Russell Toris -- rctoris@wpi.edu
     * @version December 3, 2014
     */
    public static class Response extends ServiceResponse {

        /**
         * Create a new empty LearnModels ServiceResponse.
         */
        public Response() {
            this(true);
        }

        /**
         * Create a new LearnModels ServiceResponse.
         *
         * @param result
         *            The result flag for the response (i.e., if the service
         *            server returned a success).
         */
        public Response(boolean result) {
            super(ServiceResponse.EMPTY_MESSAGE, LearnModels.TYPE, result);
        }

        /**
         * Create a clone of this LearnModels ServiceResponse.
         */
        @Override
        public Response clone() {
            return new LearnModels.Response(this.getResult());
        }

        /**
         * Create a new LearnModels ServiceResponse based on the given JSON string.
         * Any missing values will be set to their defaults.
         *
         * @param jsonString
         *            The JSON string to parse.
         * @return A LearnModels ServiceResponse based on the given JSON string.
         */
        public static Response fromJsonString(String jsonString) {
            // convert to a ServiceResponse
            return LearnModels.Response.fromServiceResponse(new ServiceResponse(
                    jsonString, true));
        }

        /**
         * Create a new LearnModels ServiceResponse based on the given
         * ServiceResponse. Any missing values will be set to their defaults.
         *
         * @param resp
         *            The ServiceResponse to parse.
         * @return A LearnModels ServiceResponse based on the given Message.
         */
        public static Response fromServiceResponse(ServiceResponse resp) {
            // get it from the JSON object
            return LearnModels.Response.fromJsonObject(resp.toJsonObject());
        }

        /**
         * Create a new LearnModels ServiceResponse based on the given JSON object.
         * Any missing values will be set to their defaults.
         *
         * @param jsonObject
         *            The JSON object to parse.
         * @return A LearnModels ServiceResponse based on the given JSON object.
         */
        public static Response fromJsonObject(JsonObject jsonObject) {
            return new LearnModels.Response(true);
        }
    }
}
