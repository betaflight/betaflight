var serial = {
    connectionId: -1,
    
    connect: function(path, options, callback) {
        var self = this;
        
        chrome.serial.connect(path, options, function(connectionInfo) {
            self.connectionId = connectionInfo.connectionId;
            callback(connectionInfo);
        });
    },
    disconnect: function(callback) {
        var self = this;
        
        chrome.serial.disconnect(this.connectionId, function(result) {
            self.connectionId = -1;
            callback(result);
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
    setControlSignals: function(signals, callback) {
        chrome.serial.setControlSignals(this.connectionId, signals, callback);
    },
    send: function(data, callback) {
        chrome.serial.send(this.connectionId, data, callback);
    },
    onReceive: {
        listeners_: chrome.serial.onReceive.listeners_,
        
        addListener: function(function_reference) {
            chrome.serial.onReceive.addListener(function_reference);
        },
        removeListener: function(function_reference) {
            chrome.serial.onReceive.removeListener(function_reference);
        }
    }
};