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

class ObstacleInfo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.has_obstacle = null;
      this.min_distance = null;
      this.front_distance = null;
      this.left_distance = null;
      this.right_distance = null;
      this.rear_distance = null;
      this.nearest_point = null;
      this.obstacle_point_count = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('has_obstacle')) {
        this.has_obstacle = initObj.has_obstacle
      }
      else {
        this.has_obstacle = false;
      }
      if (initObj.hasOwnProperty('min_distance')) {
        this.min_distance = initObj.min_distance
      }
      else {
        this.min_distance = 0.0;
      }
      if (initObj.hasOwnProperty('front_distance')) {
        this.front_distance = initObj.front_distance
      }
      else {
        this.front_distance = 0.0;
      }
      if (initObj.hasOwnProperty('left_distance')) {
        this.left_distance = initObj.left_distance
      }
      else {
        this.left_distance = 0.0;
      }
      if (initObj.hasOwnProperty('right_distance')) {
        this.right_distance = initObj.right_distance
      }
      else {
        this.right_distance = 0.0;
      }
      if (initObj.hasOwnProperty('rear_distance')) {
        this.rear_distance = initObj.rear_distance
      }
      else {
        this.rear_distance = 0.0;
      }
      if (initObj.hasOwnProperty('nearest_point')) {
        this.nearest_point = initObj.nearest_point
      }
      else {
        this.nearest_point = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('obstacle_point_count')) {
        this.obstacle_point_count = initObj.obstacle_point_count
      }
      else {
        this.obstacle_point_count = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ObstacleInfo
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [has_obstacle]
    bufferOffset = _serializer.bool(obj.has_obstacle, buffer, bufferOffset);
    // Serialize message field [min_distance]
    bufferOffset = _serializer.float64(obj.min_distance, buffer, bufferOffset);
    // Serialize message field [front_distance]
    bufferOffset = _serializer.float64(obj.front_distance, buffer, bufferOffset);
    // Serialize message field [left_distance]
    bufferOffset = _serializer.float64(obj.left_distance, buffer, bufferOffset);
    // Serialize message field [right_distance]
    bufferOffset = _serializer.float64(obj.right_distance, buffer, bufferOffset);
    // Serialize message field [rear_distance]
    bufferOffset = _serializer.float64(obj.rear_distance, buffer, bufferOffset);
    // Serialize message field [nearest_point]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.nearest_point, buffer, bufferOffset);
    // Serialize message field [obstacle_point_count]
    bufferOffset = _serializer.uint32(obj.obstacle_point_count, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ObstacleInfo
    let len;
    let data = new ObstacleInfo(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [has_obstacle]
    data.has_obstacle = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [min_distance]
    data.min_distance = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [front_distance]
    data.front_distance = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [left_distance]
    data.left_distance = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [right_distance]
    data.right_distance = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [rear_distance]
    data.rear_distance = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [nearest_point]
    data.nearest_point = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [obstacle_point_count]
    data.obstacle_point_count = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 69;
  }

  static datatype() {
    // Returns string type for a message object
    return 'fastnav_msgs/ObstacleInfo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '25281cdc1a810d2ecef8ab29564e1d71';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # FastNav obstacle information.
    # This message is published by fastnav_perception
    # and consumed by fastnav_planner.
    
    std_msgs/Header header
    
    bool has_obstacle
    
    float64 min_distance
    float64 front_distance
    float64 left_distance
    float64 right_distance
    float64 rear_distance
    
    geometry_msgs/Point nearest_point
    
    uint32 obstacle_point_count
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
    const resolved = new ObstacleInfo(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.has_obstacle !== undefined) {
      resolved.has_obstacle = msg.has_obstacle;
    }
    else {
      resolved.has_obstacle = false
    }

    if (msg.min_distance !== undefined) {
      resolved.min_distance = msg.min_distance;
    }
    else {
      resolved.min_distance = 0.0
    }

    if (msg.front_distance !== undefined) {
      resolved.front_distance = msg.front_distance;
    }
    else {
      resolved.front_distance = 0.0
    }

    if (msg.left_distance !== undefined) {
      resolved.left_distance = msg.left_distance;
    }
    else {
      resolved.left_distance = 0.0
    }

    if (msg.right_distance !== undefined) {
      resolved.right_distance = msg.right_distance;
    }
    else {
      resolved.right_distance = 0.0
    }

    if (msg.rear_distance !== undefined) {
      resolved.rear_distance = msg.rear_distance;
    }
    else {
      resolved.rear_distance = 0.0
    }

    if (msg.nearest_point !== undefined) {
      resolved.nearest_point = geometry_msgs.msg.Point.Resolve(msg.nearest_point)
    }
    else {
      resolved.nearest_point = new geometry_msgs.msg.Point()
    }

    if (msg.obstacle_point_count !== undefined) {
      resolved.obstacle_point_count = msg.obstacle_point_count;
    }
    else {
      resolved.obstacle_point_count = 0
    }

    return resolved;
    }
};

module.exports = ObstacleInfo;
