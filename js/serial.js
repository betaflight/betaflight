var serial = {
    connectionId:           -1,
    bitrate:                0,
    bytes_received:         0,
    bytes_sent:             0,

    transmitting:   false,
    output_buffer:  [],

    connect: function(path, options, callback) {
        var self = this;

        chrome.serial.connect(path, options, function(connectionInfo) {
            if (connectionInfo !== undefined) {
                self.connectionId = connectionInfo.connectionId;
                self.bitrate = connectionInfo.bitrate;
                self.bytes_received = 0;
                self.bytes_sent = 0;

                self.onReceive.addListener(function log_bytes_received(info) {
                    self.bytes_received += info.data.byteLength;
                });

                self.onReceiveError.addListener(function watch_for_on_receive_errors(info) {
                    console.error(info);
                    googleAnalytics.sendException('Serial: ' + info.error, false);

                    switch (info.error) {
                        case 'system_error': // we might be able to recover from this one
                            chrome.serial.setPaused(self.connectionId, false, get_status);

                            function get_status() {
                                self.getInfo(crunch_status);
                            }

                            function crunch_status(info) {
                                if (!info.paused) {
                                    console.log('SERIAL: Connection recovered from last onReceiveError');
                                    googleAnalytics.sendException('Serial: onReceiveError - recovered', false);
                                } else {
                                    console.log('SERIAL: Connection did not recover from last onReceiveError, disconnecting');
                                    GUI.log('Unrecoverable <span style="color: red">failure</span> of serial connection, disconnecting...');
                                    googleAnalytics.sendException('Serial: onReceiveError - unrecoverable', false);

                                    if (GUI.connected_to || GUI.connecting_to) {
                                        $('a.connect').click();
                                    } else {
                                        self.disconnect();
                                    }
                                }
                            }
                            break;
                        case 'timeout':
                            // TODO
                            break;
                        case 'device_lost':
                            // TODO
                            break;
                        case 'disconnected':
                            // TODO
                            break;
                    }
                });

                console.log('SERIAL: Connection opened with ID: ' + connectionInfo.connectionId + ', Baud: ' + connectionInfo.bitrate);

                if (callback) callback(connectionInfo);
            } else {
                console.log('SERIAL: Failed to open serial port');
                googleAnalytics.sendException('Serial: FailedToOpen', false);
                if (callback) callback(false);
            }
        });
    },
    disconnect: function(callback) {
        var self = this;

        self.empty_output_buffer();

        // remove listeners
        for (var i = (self.onReceive.listeners.length - 1); i >= 0; i--) {
            self.onReceive.removeListener(self.onReceive.listeners[i]);
        }

        for (var i = (self.onReceiveError.listeners.length - 1); i >= 0; i--) {
            self.onReceiveError.removeListener(self.onReceiveError.listeners[i]);
        }

        chrome.serial.disconnect(this.connectionId, function(result) {
            if (result) {
                console.log('SERIAL: Connection with ID: ' + self.connectionId + ' closed');
            } else {
                console.log('SERIAL: Failed to close connection with ID: ' + self.connectionId + ' closed');
                googleAnalytics.sendException('Serial: FailedToClose', false);
            }

            console.log('SERIAL: Statistics - Sent: ' + self.bytes_sent + ' bytes, Received: ' + self.bytes_received + ' bytes');

            self.connectionId = -1;
            self.bitrate = 0;

            if (callback) callback(result);
        });
    },
    getDevices: function(callback) {
        chrome.serial.getDevices(function(devices_array) {
            var devices = [];
            devices_array.forEach(function(device) {
                devices.push(device.path);
            });

            callback(devices);
        });
    },
    getInfo: function(callback) {
        chrome.serial.getInfo(this.connectionId, callback);
    },
    getControlSignals: function(callback) {
        chrome.serial.getControlSignals(this.connectionId, callback);
    },
    setControlSignals: function(signals, callback) {
        chrome.serial.setControlSignals(this.connectionId, signals, callback);
    },
    send: function(data, callback) {
        var self = this;
        self.output_buffer.push({'data': data, 'callback': callback});

        if (!self.transmitting) {
            self.transmitting = true;

            function sending() {
                // store inside separate variables in case array gets destroyed
                var data = self.output_buffer[0].data;
                var callback = self.output_buffer[0].callback;

                chrome.serial.send(self.connectionId, data, function(sendInfo) {
                    callback(sendInfo);
                    self.output_buffer.shift();

                    self.bytes_sent += sendInfo.bytesSent;

                    if (self.output_buffer.length) {
                        // keep the buffer withing reasonable limits
                        while (self.output_buffer.length > 500) {
                            self.output_buffer.pop();
                        }

                        sending();
                    } else {
                        self.transmitting = false;
                    }
                });
            };

            sending();
        }
    },
    onReceive: {
        listeners: [],

        addListener: function(function_reference) {
            var listener = chrome.serial.onReceive.addListener(function_reference);

            this.listeners.push(function_reference);
        },
        removeListener: function(function_reference) {
            for (var i = (this.listeners.length - 1); i >= 0; i--) {
                if (this.listeners[i] == function_reference) {
                    chrome.serial.onReceive.removeListener(function_reference);

                    this.listeners.splice(i, 1);
                    break;
                }
            }
        }
    },
    onReceiveError: {
        listeners: [],

        addListener: function(function_reference) {
            var listener = chrome.serial.onReceiveError.addListener(function_reference);

            this.listeners.push(function_reference);
        },
        removeListener: function(function_reference) {
            for (var i = (this.listeners.length - 1); i >= 0; i--) {
                if (this.listeners[i] == function_reference) {
                    chrome.serial.onReceiveError.removeListener(function_reference);

                    this.listeners.splice(i, 1);
                    break;
                }
            }
        }
    },
    empty_output_buffer: function() {
        this.output_buffer = [];
        this.transmitting = false;
    }
};