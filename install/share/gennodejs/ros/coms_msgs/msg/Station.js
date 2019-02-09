// Auto-generated. Do not edit!

// (in-package coms_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Station {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.headerstamp = null;
      this.xMax = null;
      this.xMin = null;
      this.yMax = null;
      this.yMin = null;
      this.zMax = null;
      this.zMin = null;
      this.flap_not_open_error = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('headerstamp')) {
        this.headerstamp = initObj.headerstamp
      }
      else {
        this.headerstamp = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('xMax')) {
        this.xMax = initObj.xMax
      }
      else {
        this.xMax = 0.0;
      }
      if (initObj.hasOwnProperty('xMin')) {
        this.xMin = initObj.xMin
      }
      else {
        this.xMin = 0.0;
      }
      if (initObj.hasOwnProperty('yMax')) {
        this.yMax = initObj.yMax
      }
      else {
        this.yMax = 0.0;
      }
      if (initObj.hasOwnProperty('yMin')) {
        this.yMin = initObj.yMin
      }
      else {
        this.yMin = 0.0;
      }
      if (initObj.hasOwnProperty('zMax')) {
        this.zMax = initObj.zMax
      }
      else {
        this.zMax = 0.0;
      }
      if (initObj.hasOwnProperty('zMin')) {
        this.zMin = initObj.zMin
      }
      else {
        this.zMin = 0.0;
      }
      if (initObj.hasOwnProperty('flap_not_open_error')) {
        this.flap_not_open_error = initObj.flap_not_open_error
      }
      else {
        this.flap_not_open_error = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Station
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [headerstamp]
    bufferOffset = _serializer.time(obj.headerstamp, buffer, bufferOffset);
    // Serialize message field [xMax]
    bufferOffset = _serializer.float32(obj.xMax, buffer, bufferOffset);
    // Serialize message field [xMin]
    bufferOffset = _serializer.float32(obj.xMin, buffer, bufferOffset);
    // Serialize message field [yMax]
    bufferOffset = _serializer.float32(obj.yMax, buffer, bufferOffset);
    // Serialize message field [yMin]
    bufferOffset = _serializer.float32(obj.yMin, buffer, bufferOffset);
    // Serialize message field [zMax]
    bufferOffset = _serializer.float32(obj.zMax, buffer, bufferOffset);
    // Serialize message field [zMin]
    bufferOffset = _serializer.float32(obj.zMin, buffer, bufferOffset);
    // Serialize message field [flap_not_open_error]
    bufferOffset = _serializer.bool(obj.flap_not_open_error, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Station
    let len;
    let data = new Station(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [headerstamp]
    data.headerstamp = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [xMax]
    data.xMax = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [xMin]
    data.xMin = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [yMax]
    data.yMax = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [yMin]
    data.yMin = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [zMax]
    data.zMax = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [zMin]
    data.zMin = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [flap_not_open_error]
    data.flap_not_open_error = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 33;
  }

  static datatype() {
    // Returns string type for a message object
    return 'coms_msgs/Station';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c279ad04cbc81207a5714883b84da036';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    time headerstamp
    float32 xMax
    float32 xMin
    float32 yMax
    float32 yMin
    float32 zMax
    float32 zMin
    bool flap_not_open_error
    
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
    # 0: no frame
    # 1: global frame
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Station(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.headerstamp !== undefined) {
      resolved.headerstamp = msg.headerstamp;
    }
    else {
      resolved.headerstamp = {secs: 0, nsecs: 0}
    }

    if (msg.xMax !== undefined) {
      resolved.xMax = msg.xMax;
    }
    else {
      resolved.xMax = 0.0
    }

    if (msg.xMin !== undefined) {
      resolved.xMin = msg.xMin;
    }
    else {
      resolved.xMin = 0.0
    }

    if (msg.yMax !== undefined) {
      resolved.yMax = msg.yMax;
    }
    else {
      resolved.yMax = 0.0
    }

    if (msg.yMin !== undefined) {
      resolved.yMin = msg.yMin;
    }
    else {
      resolved.yMin = 0.0
    }

    if (msg.zMax !== undefined) {
      resolved.zMax = msg.zMax;
    }
    else {
      resolved.zMax = 0.0
    }

    if (msg.zMin !== undefined) {
      resolved.zMin = msg.zMin;
    }
    else {
      resolved.zMin = 0.0
    }

    if (msg.flap_not_open_error !== undefined) {
      resolved.flap_not_open_error = msg.flap_not_open_error;
    }
    else {
      resolved.flap_not_open_error = false
    }

    return resolved;
    }
};

module.exports = Station;
