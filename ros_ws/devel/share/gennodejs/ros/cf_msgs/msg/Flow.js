// Auto-generated. Do not edit!

// (in-package cf_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Flow {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.deltaX = null;
      this.deltaY = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('deltaX')) {
        this.deltaX = initObj.deltaX
      }
      else {
        this.deltaX = 0;
      }
      if (initObj.hasOwnProperty('deltaY')) {
        this.deltaY = initObj.deltaY
      }
      else {
        this.deltaY = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Flow
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [deltaX]
    bufferOffset = _serializer.int32(obj.deltaX, buffer, bufferOffset);
    // Serialize message field [deltaY]
    bufferOffset = _serializer.int32(obj.deltaY, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Flow
    let len;
    let data = new Flow(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [deltaX]
    data.deltaX = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [deltaY]
    data.deltaY = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'cf_msgs/Flow';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1355de43fdad329d668323cf3db6c71c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    int32 deltaX
    int32 deltaY
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
    const resolved = new Flow(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.deltaX !== undefined) {
      resolved.deltaX = msg.deltaX;
    }
    else {
      resolved.deltaX = 0
    }

    if (msg.deltaY !== undefined) {
      resolved.deltaY = msg.deltaY;
    }
    else {
      resolved.deltaY = 0
    }

    return resolved;
    }
};

module.exports = Flow;
