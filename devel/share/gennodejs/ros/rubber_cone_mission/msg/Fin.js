// Auto-generated. Do not edit!

// (in-package rubber_cone_mission.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Fin {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.num = null;
      this.centroid_x = null;
      this.centroid_y = null;
      this.aveD = null;
      this.minD = null;
      this.angle = null;
      this.size = null;
    }
    else {
      if (initObj.hasOwnProperty('num')) {
        this.num = initObj.num
      }
      else {
        this.num = 0;
      }
      if (initObj.hasOwnProperty('centroid_x')) {
        this.centroid_x = initObj.centroid_x
      }
      else {
        this.centroid_x = [];
      }
      if (initObj.hasOwnProperty('centroid_y')) {
        this.centroid_y = initObj.centroid_y
      }
      else {
        this.centroid_y = [];
      }
      if (initObj.hasOwnProperty('aveD')) {
        this.aveD = initObj.aveD
      }
      else {
        this.aveD = [];
      }
      if (initObj.hasOwnProperty('minD')) {
        this.minD = initObj.minD
      }
      else {
        this.minD = [];
      }
      if (initObj.hasOwnProperty('angle')) {
        this.angle = initObj.angle
      }
      else {
        this.angle = [];
      }
      if (initObj.hasOwnProperty('size')) {
        this.size = initObj.size
      }
      else {
        this.size = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Fin
    // Serialize message field [num]
    bufferOffset = _serializer.int64(obj.num, buffer, bufferOffset);
    // Serialize message field [centroid_x]
    bufferOffset = _arraySerializer.float64(obj.centroid_x, buffer, bufferOffset, null);
    // Serialize message field [centroid_y]
    bufferOffset = _arraySerializer.float64(obj.centroid_y, buffer, bufferOffset, null);
    // Serialize message field [aveD]
    bufferOffset = _arraySerializer.float64(obj.aveD, buffer, bufferOffset, null);
    // Serialize message field [minD]
    bufferOffset = _arraySerializer.float64(obj.minD, buffer, bufferOffset, null);
    // Serialize message field [angle]
    bufferOffset = _arraySerializer.float64(obj.angle, buffer, bufferOffset, null);
    // Serialize message field [size]
    bufferOffset = _arraySerializer.float64(obj.size, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Fin
    let len;
    let data = new Fin(null);
    // Deserialize message field [num]
    data.num = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [centroid_x]
    data.centroid_x = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [centroid_y]
    data.centroid_y = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [aveD]
    data.aveD = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [minD]
    data.minD = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [angle]
    data.angle = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [size]
    data.size = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.centroid_x.length;
    length += 8 * object.centroid_y.length;
    length += 8 * object.aveD.length;
    length += 8 * object.minD.length;
    length += 8 * object.angle.length;
    length += 8 * object.size.length;
    return length + 32;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rubber_cone_mission/Fin';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '421d0d272a8d8a52e65daee64dcb6ee2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64 num            # 유효한 클러스터의 개수
    float64[] centroid_x # 유효한 클러스터들의 x 좌표
    float64[] centroid_y # 유효한 클러스터들의 y 좌표
    float64[] aveD    # 각 클러스터의 중심점 x 좌표 리스트
    float64[] minD    # 각 클러스터의 중심점 y 좌표 리스트
    float64[] angle   # 각 클러스터의 각도 (필요한 경우)
    float64[] size    # 각 클러스터의 크기 (필요한 경우)
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Fin(null);
    if (msg.num !== undefined) {
      resolved.num = msg.num;
    }
    else {
      resolved.num = 0
    }

    if (msg.centroid_x !== undefined) {
      resolved.centroid_x = msg.centroid_x;
    }
    else {
      resolved.centroid_x = []
    }

    if (msg.centroid_y !== undefined) {
      resolved.centroid_y = msg.centroid_y;
    }
    else {
      resolved.centroid_y = []
    }

    if (msg.aveD !== undefined) {
      resolved.aveD = msg.aveD;
    }
    else {
      resolved.aveD = []
    }

    if (msg.minD !== undefined) {
      resolved.minD = msg.minD;
    }
    else {
      resolved.minD = []
    }

    if (msg.angle !== undefined) {
      resolved.angle = msg.angle;
    }
    else {
      resolved.angle = []
    }

    if (msg.size !== undefined) {
      resolved.size = msg.size;
    }
    else {
      resolved.size = []
    }

    return resolved;
    }
};

module.exports = Fin;
