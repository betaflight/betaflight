'use strict';

var MSP = {
    state:                      0,
    message_direction:          1,
    code:                       0,
    dataView:                   0,
    message_length_expected:    0,
    message_length_received:    0,
    message_buffer:             null,
    message_buffer_uint8_view:  null,
    message_checksum:           0,
    messageIsJumboFrame:        false,

    callbacks:                  [],
    packet_error:               0,
    unsupported:                0,

    last_received_timestamp:   null,
    listeners:                  [],

    JUMBO_FRAME_SIZE_LIMIT:     255,
    
    read: function (readInfo) {
        var data = new Uint8Array(readInfo.data);

        for (var i = 0; i < data.length; i++) {
            switch (this.state) {
                case 0: // sync char 1
                    if (data[i] == 36) { // $
                        this.state++;
                    }
                    break;
                case 1: // sync char 2
                    if (data[i] == 77) { // M
                        this.state++;
                    } else { // restart and try again
                        this.state = 0;
                    }
                    break;
                case 2: // direction (should be >)
                    this.unsupported = 0;
                    if (data[i] == 62) { // >
                        this.message_direction = 1;
                    } else if (data[i] == 60) { // <
                        this.message_direction = 0;
                    } else if (data[i] == 33) { // !
                        // FC reports unsupported message error
                        this.unsupported = 1;
                    }

                    this.state++;
                    break;
                case 3:
                    this.message_length_expected = data[i];
                    if (this.message_length_expected === this.JUMBO_FRAME_SIZE_LIMIT) {
                        this.messageIsJumboFrame = true;
                    }

                    this.message_checksum = data[i];

                    this.state++;
                    break;
                case 4:
                    this.code = data[i];
                    this.message_checksum ^= data[i];

                    if (this.message_length_expected > 0) {
                        // process payload
                        if (this.messageIsJumboFrame) {
                            this.state++;
                        } else {
                            this.state = this.state + 3;
                        }
                    } else {
                        // no payload
                        this.state += 5;
                    }
                    break;
                case 5:
                    this.message_length_expected = data[i];

                    this.message_checksum ^= data[i];

                    this.state++;

                    break;
                case 6:
                    this.message_length_expected = this.message_length_expected  + 256 * data[i];

                    this.message_checksum ^= data[i];

                    this.state++;

                    break;
                case 7:
                    // setup arraybuffer
                    this.message_buffer = new ArrayBuffer(this.message_length_expected);
                    this.message_buffer_uint8_view = new Uint8Array(this.message_buffer);

                    this.state++;
                case 8: // payload
                    this.message_buffer_uint8_view[this.message_length_received] = data[i];
                    this.message_checksum ^= data[i];
                    this.message_length_received++;

                    if (this.message_length_received >= this.message_length_expected) {
                        this.state++;
                    }
                    break;
                case 9:
                    if (this.message_checksum == data[i]) {
                        // message received, store dataview
                        this.dataView = new DataView(this.message_buffer, 0, this.message_length_expected);
                    } else {
                        console.log('code: ' + this.code + ' - crc failed');
                        this.dataView = null;
                        this.packet_error++;
                    }
                    // Reset variables
                    this.message_length_received = 0;
                    this.state = 0;
                    this.messageIsJumboFrame = false;
                    this.notify();
                    break;

                default:
                    console.log('Unknown state detected: ' + this.state);
            }
        }
        this.last_received_timestamp = Date.now();
    },
    notify: function() {
        var self = this;
        this.listeners.forEach(function(listener) {
            listener(self);
        });
    },
    listen: function(listener) {
        if (this.listeners.indexOf(listener) == -1) {
            this.listeners.push(listener);
        }
    },
    clearListeners: function() {
        this.listeners = [];  
    },
    send_message: function (code, data, callback_sent, callback_msp) {
        var bufferOut,
            bufView;

        // always reserve 6 bytes for protocol overhead !
        if (data) {
            var size = data.length + 6,
                checksum = 0;

            bufferOut = new ArrayBuffer(size);
            bufView = new Uint8Array(bufferOut);

            bufView[0] = 36; // $
            bufView[1] = 77; // M
            bufView[2] = 60; // <
            bufView[3] = data.length;
            bufView[4] = code;

            checksum = bufView[3] ^ bufView[4];

            for (var i = 0; i < data.length; i++) {
                bufView[i + 5] = data[i];

                checksum ^= bufView[i + 5];
            }

            bufView[5 + data.length] = checksum;
        } else {
            bufferOut = new ArrayBuffer(6);
            bufView = new Uint8Array(bufferOut);

            bufView[0] = 36; // $
            bufView[1] = 77; // M
            bufView[2] = 60; // <
            bufView[3] = 0; // data length
            bufView[4] = code; // code
            bufView[5] = bufView[3] ^ bufView[4]; // checksum
        }

        var obj = {'code': code, 'requestBuffer': bufferOut, 'callback': (callback_msp) ? callback_msp : false, 'timer': false};

        var requestExists = false;
        for (var i = 0; i < MSP.callbacks.length; i++) {
            if (MSP.callbacks[i].code == code) {
                // request already exist, we will just attach
                requestExists = true;
                break;
            }
        }

        if (!requestExists) {
            obj.timer = setInterval(function () {
                console.log('MSP data request timed-out: ' + code);

                serial.send(bufferOut, false);
            }, 1000); // we should be able to define timeout in the future
        }

        MSP.callbacks.push(obj);

        // always send messages with data payload (even when there is a message already in the queue)
        if (data || !requestExists) {
            serial.send(bufferOut, function (sendInfo) {
                if (sendInfo.bytesSent == bufferOut.byteLength) {
                    if (callback_sent) callback_sent();
                }
            });
        }

        return true;
    },
    /**
     * resolves: {command: code, data: data, length: message_length}
     */
    promise: function(code, data) {
      var self = this;
      return new Promise(function(resolve) {
        self.send_message(code, data, false, function(data) {
          resolve(data);
        });
      });
    },
    callbacks_cleanup: function () {
        for (var i = 0; i < this.callbacks.length; i++) {
            clearInterval(this.callbacks[i].timer);
        }

        this.callbacks = [];
    },
    disconnect_cleanup: function () {
        this.state = 0; // reset packet state for "clean" initial entry (this is only required if user hot-disconnects)
        this.packet_error = 0; // reset CRC packet error counter for next session

        this.callbacks_cleanup();
    }
};
