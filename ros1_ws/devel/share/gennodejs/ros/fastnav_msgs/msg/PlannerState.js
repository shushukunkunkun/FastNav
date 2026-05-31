// Auto-generated. Do not edit!

// (in-package fastnav_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class PlannerState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.state = null;
      this.state_name = null;
      this.current_goal = null;
      this.goal_reached = null;
      this.has_obstacle = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('state')) {
        this.state = initObj.state
      }
      else {
        this.state = 0;
      }
      if (initObj.hasOwnProperty('state_name')) {
        this.state_name = initObj.state_name
      }
      else {
        this.state_name = '';
      }
      if (initObj.hasOwnProperty('current_goal')) {
        this.current_goal = initObj.current_goal
      }
      else {
        this.current_goal = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('goal_reached')) {
        this.goal_reached = initObj.goal_reached
      }
      else {
        this.goal_reached = false;
      }
      if (initObj.hasOwnProperty('has_obstacle')) {
        this.has_obstacle = initObj.has_obstacle
      }
      else {
        this.has_obstacle = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PlannerState
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [state]
    bufferOffset = _serializer.uint8(obj.state, buffer, bufferOffset);
    // Serialize message field [state_name]
    bufferOffset = _serializer.string(obj.state_name, buffer, bufferOffset);
    // Serialize message field [current_goal]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.current_goal, buffer, bufferOffset);
    // Serialize message field [goal_reached]
    bufferOffset = _serializer.bool(obj.goal_reached, buffer, bufferOffset);
    // Serialize message field [has_obstacle]
    bufferOffset = _serializer.bool(obj.has_obstacle, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PlannerState
    let len;
    let data = new PlannerState(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [state]
    data.state = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [state_name]
    data.state_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [current_goal]
    data.current_goal = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [goal_reached]
    data.goal_reached = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [has_obstacle]
    data.has_obstacle = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.state_name);
    return length + 31;
  }

  static datatype() {
    // Returns string type for a message object
    return 'fastnav_msgs/PlannerState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2f756c3d9a0a66ce8958743e6cc2f690';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # FastNav planner state.
    # This message is published by fastnav_planner
    # for debugging and visualization.
    
    std_msgs/Header header
    
    uint8 STATE_IDLE        = 0
    uint8 STATE_TAKEOFF     = 1
    uint8 STATE_NAVIGATE    = 2
    uint8 STATE_AVOID       = 3
    uint8 STATE_HOVER       = 4
    uint8 STATE_GOAL_REACHED = 5
    uint8 STATE_EMERGENCY   = 6
    
    uint8 state
    string state_name
    
    geometry_msgs/Point current_goal
    
    bool goal_reached
    bool has_obstacle
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PlannerState(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.state !== undefined) {
      resolved.state = msg.state;
    }
    else {
      resolved.state = 0
    }

    if (msg.state_name !== undefined) {
      resolved.state_name = msg.state_name;
    }
    else {
      resolved.state_name = ''
    }

    if (msg.current_goal !== undefined) {
      resolved.current_goal = geometry_msgs.msg.Point.Resolve(msg.current_goal)
    }
    else {
      resolved.current_goal = new geometry_msgs.msg.Point()
    }

    if (msg.goal_reached !== undefined) {
      resolved.goal_reached = msg.goal_reached;
    }
    else {
      resolved.goal_reached = false
    }

    if (msg.has_obstacle !== undefined) {
      resolved.has_obstacle = msg.has_obstacle;
    }
    else {
      resolved.has_obstacle = false
    }

    return resolved;
    }
};

// Constants for message
PlannerState.Constants = {
  STATE_IDLE: 0,
  STATE_TAKEOFF: 1,
  STATE_NAVIGATE: 2,
  STATE_AVOID: 3,
  STATE_HOVER: 4,
  STATE_GOAL_REACHED: 5,
  STATE_EMERGENCY: 6,
}

module.exports = PlannerState;
