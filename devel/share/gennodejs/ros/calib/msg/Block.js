// Auto-generated. Do not edit!

// (in-package calib.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Block {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.block_id = null;
      this.block_x = null;
      this.block_y = null;
    }
    else {
      if (initObj.hasOwnProperty('block_id')) {
        this.block_id = initObj.block_id
      }
      else {
        this.block_id = [];
      }
      if (initObj.hasOwnProperty('block_x')) {
        this.block_x = initObj.block_x
      }
      else {
        this.block_x = [];
      }
      if (initObj.hasOwnProperty('block_y')) {
        this.block_y = initObj.block_y
      }
      else {
        this.block_y = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Block
    // Serialize message field [block_id]
    bufferOffset = _arraySerializer.float32(obj.block_id, buffer, bufferOffset, null);
    // Serialize message field [block_x]
    bufferOffset = _arraySerializer.float32(obj.block_x, buffer, bufferOffset, null);
    // Serialize message field [block_y]
    bufferOffset = _arraySerializer.float32(obj.block_y, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Block
    let len;
    let data = new Block(null);
    // Deserialize message field [block_id]
    data.block_id = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [block_x]
    data.block_x = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [block_y]
    data.block_y = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.block_id.length;
    length += 4 * object.block_x.length;
    length += 4 * object.block_y.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'calib/Block';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9643fae418351e15e5ed3fde4997c5fa';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32[] block_id
    float32[] block_x
    float32[] block_y
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Block(null);
    if (msg.block_id !== undefined) {
      resolved.block_id = msg.block_id;
    }
    else {
      resolved.block_id = []
    }

    if (msg.block_x !== undefined) {
      resolved.block_x = msg.block_x;
    }
    else {
      resolved.block_x = []
    }

    if (msg.block_y !== undefined) {
      resolved.block_y = msg.block_y;
    }
    else {
      resolved.block_y = []
    }

    return resolved;
    }
};

module.exports = Block;
