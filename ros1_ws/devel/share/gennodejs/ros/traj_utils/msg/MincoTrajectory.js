// Auto-generated. Do not edit!

// (in-package traj_utils.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class MincoTrajectory {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.traj_id = null;
      this.order = null;
      this.start_time = null;
      this.duration = null;
      this.coef_x = null;
      this.coef_y = null;
      this.coef_z = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('traj_id')) {
        this.traj_id = initObj.traj_id
      }
      else {
        this.traj_id = 0;
      }
      if (initObj.hasOwnProperty('order')) {
        this.order = initObj.order
      }
      else {
        this.order = 0;
      }
      if (initObj.hasOwnProperty('start_time')) {
        this.start_time = initObj.start_time
      }
      else {
        this.start_time = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('duration')) {
        this.duration = initObj.duration
      }
      else {
        this.duration = [];
      }
      if (initObj.hasOwnProperty('coef_x')) {
        this.coef_x = initObj.coef_x
      }
      else {
        this.coef_x = [];
      }
      if (initObj.hasOwnProperty('coef_y')) {
        this.coef_y = initObj.coef_y
      }
      else {
        this.coef_y = [];
      }
      if (initObj.hasOwnProperty('coef_z')) {
        this.coef_z = initObj.coef_z
      }
      else {
        this.coef_z = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MincoTrajectory
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [traj_id]
    bufferOffset = _serializer.uint32(obj.traj_id, buffer, bufferOffset);
    // Serialize message field [order]
    bufferOffset = _serializer.int32(obj.order, buffer, bufferOffset);
    // Serialize message field [start_time]
    bufferOffset = _serializer.time(obj.start_time, buffer, bufferOffset);
    // Serialize message field [duration]
    bufferOffset = _arraySerializer.float64(obj.duration, buffer, bufferOffset, null);
    // Serialize message field [coef_x]
    bufferOffset = _arraySerializer.float64(obj.coef_x, buffer, bufferOffset, null);
    // Serialize message field [coef_y]
    bufferOffset = _arraySerializer.float64(obj.coef_y, buffer, bufferOffset, null);
    // Serialize message field [coef_z]
    bufferOffset = _arraySerializer.float64(obj.coef_z, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MincoTrajectory
    let len;
    let data = new MincoTrajectory(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [traj_id]
    data.traj_id = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [order]
    data.order = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [start_time]
    data.start_time = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [duration]
    data.duration = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [coef_x]
    data.coef_x = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [coef_y]
    data.coef_y = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [coef_z]
    data.coef_z = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 8 * object.duration.length;
    length += 8 * object.coef_x.length;
    length += 8 * object.coef_y.length;
    length += 8 * object.coef_z.length;
    return length + 32;
  }

  static datatype() {
    // Returns string type for a message object
    return 'traj_utils/MincoTrajectory';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'de89d3d619bcfb3593e56bbf412d2768';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # FastNav MINCO trajectory message.
    # Planner publishes this message, traj_utils/minco_traj_server samples it into fastnav_msgs/ControlCommand.
    
    std_msgs/Header header
    
    # Monotonic trajectory id generated by planner.
    uint32 traj_id
    
    # Polynomial order. FastNav currently uses GCOPTER Trajectory<5>, so order should be 5.
    int32 order
    
    # Trajectory start time in the planner frame.
    time start_time
    
    # Piece durations. For $N$ pieces, duration has length $N$.
    float64[] duration
    
    # Per-axis coefficients of each piece.
    # For order 5, each piece owns 6 coefficients. Coefficients are stored in the same order as GCOPTER Piece<5>::CoefficientMat columns.
    # That is, coeff_x[i * 6 + k] is the x-axis coefficient at column $k$ of piece $i$.
    float64[] coef_x
    float64[] coef_y
    float64[] coef_z
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MincoTrajectory(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.traj_id !== undefined) {
      resolved.traj_id = msg.traj_id;
    }
    else {
      resolved.traj_id = 0
    }

    if (msg.order !== undefined) {
      resolved.order = msg.order;
    }
    else {
      resolved.order = 0
    }

    if (msg.start_time !== undefined) {
      resolved.start_time = msg.start_time;
    }
    else {
      resolved.start_time = {secs: 0, nsecs: 0}
    }

    if (msg.duration !== undefined) {
      resolved.duration = msg.duration;
    }
    else {
      resolved.duration = []
    }

    if (msg.coef_x !== undefined) {
      resolved.coef_x = msg.coef_x;
    }
    else {
      resolved.coef_x = []
    }

    if (msg.coef_y !== undefined) {
      resolved.coef_y = msg.coef_y;
    }
    else {
      resolved.coef_y = []
    }

    if (msg.coef_z !== undefined) {
      resolved.coef_z = msg.coef_z;
    }
    else {
      resolved.coef_z = []
    }

    return resolved;
    }
};

module.exports = MincoTrajectory;
