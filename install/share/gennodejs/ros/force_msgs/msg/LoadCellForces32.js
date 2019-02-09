// Auto-generated. Do not edit!

// (in-package force_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class LoadCellForces32 {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.headerstamp = null;
      this.cellA = null;
      this.cellB = null;
      this.cellC = null;
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
      if (initObj.hasOwnProperty('cellA')) {
        this.cellA = initObj.cellA
      }
      else {
        this.cellA = 0.0;
      }
      if (initObj.hasOwnProperty('cellB')) {
        this.cellB = initObj.cellB
      }
      else {
        this.cellB = 0.0;
      }
      if (initObj.hasOwnProperty('cellC')) {
        this.cellC = initObj.cellC
      }
      else {
        this.cellC = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type LoadCellForces32
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [headerstamp]
    bufferOffset = _serializer.time(obj.headerstamp, buffer, bufferOffset);
    // Serialize message field [cellA]
    bufferOffset = _serializer.float32(obj.cellA, buffer, bufferOffset);
    // Serialize message field [cellB]
    bufferOffset = _serializer.float32(obj.cellB, buffer, bufferOffset);
    // Serialize message field [cellC]
    bufferOffset = _serializer.float32(obj.cellC, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type LoadCellForces32
    let len;
    let data = new LoadCellForces32(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [headerstamp]
    data.headerstamp = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [cellA]
    data.cellA = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cellB]
    data.cellB = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cellC]
    data.cellC = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'force_msgs/LoadCellForces32';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f212bc1b7495a187d9c991f6bd06e446';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    time headerstamp
    float32 cellA
    float32 cellB
    float32 cellC
    
    
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
    const resolved = new LoadCellForces32(null);
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

    if (msg.cellA !== undefined) {
      resolved.cellA = msg.cellA;
    }
    else {
      resolved.cellA = 0.0
    }

    if (msg.cellB !== undefined) {
      resolved.cellB = msg.cellB;
    }
    else {
      resolved.cellB = 0.0
    }

    if (msg.cellC !== undefined) {
      resolved.cellC = msg.cellC;
    }
    else {
      resolved.cellC = 0.0
    }

    return resolved;
    }
};

module.exports = LoadCellForces32;
