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
