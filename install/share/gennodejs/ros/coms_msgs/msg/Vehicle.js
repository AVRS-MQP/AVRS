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

class Vehicle {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.headerstamp = null;
      this.model = null;
      this.charger_type = null;
      this.battery_charge = null;
      this.charge_level = null;
      this.flap_unlocked = null;
      this.flap_auto_open = null;
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
      if (initObj.hasOwnProperty('model')) {
        this.model = initObj.model
      }
      else {
        this.model = '';
      }
      if (initObj.hasOwnProperty('charger_type')) {
        this.charger_type = initObj.charger_type
      }
      else {
        this.charger_type = '';
      }
      if (initObj.hasOwnProperty('battery_charge')) {
        this.battery_charge = initObj.battery_charge
      }
      else {
        this.battery_charge = 0.0;
      }
      if (initObj.hasOwnProperty('charge_level')) {
        this.charge_level = initObj.charge_level
      }
      else {
        this.charge_level = 0;
      }
      if (initObj.hasOwnProperty('flap_unlocked')) {
        this.flap_unlocked = initObj.flap_unlocked
      }
      else {
        this.flap_unlocked = false;
      }
      if (initObj.hasOwnProperty('flap_auto_open')) {
        this.flap_auto_open = initObj.flap_auto_open
      }
      else {
        this.flap_auto_open = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Vehicle
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [headerstamp]
    bufferOffset = _serializer.time(obj.headerstamp, buffer, bufferOffset);
    // Serialize message field [model]
    bufferOffset = _serializer.string(obj.model, buffer, bufferOffset);
    // Serialize message field [charger_type]
    bufferOffset = _serializer.string(obj.charger_type, buffer, bufferOffset);
    // Serialize message field [battery_charge]
    bufferOffset = _serializer.float32(obj.battery_charge, buffer, bufferOffset);
    // Serialize message field [charge_level]
    bufferOffset = _serializer.int32(obj.charge_level, buffer, bufferOffset);
    // Serialize message field [flap_unlocked]
    bufferOffset = _serializer.bool(obj.flap_unlocked, buffer, bufferOffset);
    // Serialize message field [flap_auto_open]
    bufferOffset = _serializer.bool(obj.flap_auto_open, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Vehicle
    let len;
    let data = new Vehicle(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [headerstamp]
    data.headerstamp = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [model]
    data.model = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [charger_type]
    data.charger_type = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [battery_charge]
    data.battery_charge = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [charge_level]
    data.charge_level = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [flap_unlocked]
    data.flap_unlocked = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [flap_auto_open]
    data.flap_auto_open = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.model.length;
    length += object.charger_type.length;
    return length + 26;
  }

  static datatype() {
    // Returns string type for a message object
    return 'coms_msgs/Vehicle';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '15c570864c534a992bcad74ac9993ed9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    time headerstamp
    string model
    string charger_type
    float32 battery_charge
    int32 charge_level
    bool flap_unlocked
    bool flap_auto_open
    
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
    const resolved = new Vehicle(null);
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

    if (msg.model !== undefined) {
      resolved.model = msg.model;
    }
    else {
      resolved.model = ''
    }

    if (msg.charger_type !== undefined) {
      resolved.charger_type = msg.charger_type;
    }
    else {
      resolved.charger_type = ''
    }

    if (msg.battery_charge !== undefined) {
      resolved.battery_charge = msg.battery_charge;
    }
    else {
      resolved.battery_charge = 0.0
    }

    if (msg.charge_level !== undefined) {
      resolved.charge_level = msg.charge_level;
    }
    else {
      resolved.charge_level = 0
    }

    if (msg.flap_unlocked !== undefined) {
      resolved.flap_unlocked = msg.flap_unlocked;
    }
    else {
      resolved.flap_unlocked = false
    }

    if (msg.flap_auto_open !== undefined) {
      resolved.flap_auto_open = msg.flap_auto_open;
    }
    else {
      resolved.flap_auto_open = false
    }

    return resolved;
    }
};

module.exports = Vehicle;
