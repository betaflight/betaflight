Array.prototype.push8 = function(val) {
  this.push(0xFF & val);
  return this;
};
Array.prototype.push16 = function(val) {
  // low byte
  this.push(0x00FF & val);
  // high byte
  this.push(val >> 8);
  // chainable
  return this;
};
Array.prototype.push32 = function(val) {
    this.push8(val)
        .push8(val >> 8)
        .push8(val >> 16)
        .push8(val >> 24);
    return this;
}
DataView.prototype.offset = 0;
DataView.prototype.readU8 = function() {
    if (this.byteLength >= this.offset+1) {
        return this.getUint8(this.offset++);
    } else {
        return null;
    }
}
DataView.prototype.readU16 = function() {
    if (this.byteLength >= this.offset+2) {
        return this.readU8() + this.readU8()*256;
    } else {
        return null;
    }
}
DataView.prototype.readU32 = function() {
    if (this.byteLength >= this.offset+4) {
        return this.readU16() + this.readU16()*65536;
    } else {
        return null;
    }
}
DataView.prototype.read8 = function() {
    if (this.byteLength >= this.offset+1) {
        return this.getInt8(this.offset++, 1);
    } else {
        return null;
    }
}
DataView.prototype.read16 = function() {
    this.offset += 2;
    if (this.byteLength >= this.offset) {
        return this.getInt16(this.offset-2, 1); 
    } else {
        return null;
    }
}
DataView.prototype.read32 = function() {
    this.offset += 4;
    if (this.byteLength >= this.offset) {
        return this.getInt32(this.offset-4, 1); 
    } else {
        return null;
    }
}