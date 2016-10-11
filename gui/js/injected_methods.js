Number.prototype.clamp = function(min, max) {
    return Math.min(Math.max(this, min), max);
};

/**
 * String formatting now supports currying (partial application).
 * For a format string with N replacement indices, you can call .format
 * with M <= N arguments. The result is going to be a format string
 * with N-M replacement indices, properly counting from 0 .. N-M.
 * The following Example should explain the usage of partial applied format:
 *  "{0}:{1}:{2}".format("a","b","c") === "{0}:{1}:{2}".format("a","b").format("c")
 *  "{0}:{1}:{2}".format("a").format("b").format("c") === "{0}:{1}:{2}".format("a").format("b", "c")
 **/
String.prototype.format = function () {
    var args = arguments;
    return this.replace(/\{(\d+)\}/g, function (t, i) {
        return args[i] !== void 0 ? args[i] : "{"+(i-args.length)+"}";
    });
};

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
};

DataView.prototype.offset = 0;
DataView.prototype.readU8 = function() {
    if (this.byteLength >= this.offset+1) {
        return this.getUint8(this.offset++);
    } else {
        return null;
    }
};

DataView.prototype.readU16 = function() {
    if (this.byteLength >= this.offset+2) {
        return this.readU8() + this.readU8()*256;
    } else {
        return null;
    }
};

DataView.prototype.readU32 = function() {
    if (this.byteLength >= this.offset+4) {
        return this.readU16() + this.readU16()*65536;
    } else {
        return null;
    }
};

DataView.prototype.read8 = function() {
    if (this.byteLength >= this.offset+1) {
        return this.getInt8(this.offset++, 1);
    } else {
        return null;
    }
};

DataView.prototype.read16 = function() {
    this.offset += 2;
    if (this.byteLength >= this.offset) {
        return this.getInt16(this.offset-2, 1); 
    } else {
        return null;
    }
};

DataView.prototype.read32 = function() {
    this.offset += 4;
    if (this.byteLength >= this.offset) {
        return this.getInt32(this.offset-4, 1); 
    } else {
        return null;
    }
};