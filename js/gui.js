var GUI_control = function() {
    this.active_tab;
    
    this.interval_array = [];
    this.timeout_array = [];
};

// Timer managing methods

// name = string
// code = function reference (code to be executed)
// interval = time interval in miliseconds
// first = true/false if code should be ran initially before next timer interval hits
GUI_control.prototype.interval_add = function(name, code, interval, first) {
    var data = {'name': name, 'timer': undefined, 'interval': interval, 'fired' : 0};
    
    if (first == true) {
        code(); // execute code
        
        data.fired++; // increment counter
    }
    
    data.timer = setInterval(function() {
        code(); // execute code
        
        data.fired++; // increment counter
    }, interval);
    
    this.interval_array.push(data); // push to primary interval array
};

// name = string
GUI_control.prototype.interval_remove = function(name) {
    for (var i = 0; i < this.interval_array.length; i++) {
        if (this.interval_array[i].name == name) {
            clearInterval(this.interval_array[i].timer); // stop timer
            
            this.interval_array.splice(i, 1); // remove element/object from array
        
            return true;
        }
    }
    
    return false;
};

// input = array of timers thats meant to be kept
// return = returns timers killed in last call
GUI_control.prototype.interval_kill_all = function(keep_array) {
    var timers_killed = 0;
    
    for (var i = (this.interval_array.length - 1); i >= 0; i--) { // reverse iteration
        var self = this;
        
        var keep = false;
        keep_array.forEach(function(name) {
            if (self.interval_array[i].name == name) {
                keep = true;
            }
        });
        
        if (!keep) {
            clearInterval(this.interval_array[i].timer); // stop timer
            this.interval_array[i].timer = undefined; // set timer property to undefined (mostly for debug purposes, but it doesn't hurt to have it here)
            
            this.interval_array.splice(i, 1); // remove element/object from array
            
            timers_killed++;
        }
    }
    
    return timers_killed;
};

// name = string
// code = function reference (code to be executed)
// timeout = timeout in miliseconds
GUI_control.prototype.timeout_add = function(name, code, timeout) {
    var self = this;
    // start timer with "cleaning" callback
    var timer = setTimeout(function() {
        code(); // execute code
        
        self.timeout_remove(name); // cleanup
    }, timeout);
    
    this.timeout_array.push({'name': name, 'timer': timer, 'timeout': timeout}); // push to primary timeout array
};

// name = string
GUI_control.prototype.timeout_remove = function(name) {
    for (var i = 0; i < this.timeout_array.length; i++) {
        if (this.timeout_array[i].name == name) {
            clearTimeout(this.timeout_array[i].timer); // stop timer
            
            this.timeout_array.splice(i, 1); // remove element/object from array
            
            return true;
        }
    }
    
    return false;
};

// no input paremeters
// return = returns timers killed in last call
GUI_control.prototype.timeout_kill_all = function() {
    var timers_killed = 0;
    
    for (var i = 0; i < this.timeout_array.length; i++) {
        clearTimeout(this.timeout_array[i].timer); // stop timer
        
        timers_killed++;
    }
    
    this.timeout_array = []; // drop objects
    
    return timers_killed;
};

// Method is called every time a valid tab change event is received
// callback = code to run when cleanup is finished
// default switch doesn't require callback to be set
GUI_control.prototype.tab_switch_cleanup = function(callback) {
    switch (this.active_tab) {
        case 'cli':
            var bufferOut = new ArrayBuffer(5);
            var bufView = new Uint8Array(bufferOut);
            
            bufView[0] = 0x65; // e
            bufView[1] = 0x78; // x
            bufView[2] = 0x69; // i
            bufView[3] = 0x74; // t
            bufView[4] = 0x0D; // enter

            chrome.serial.write(connectionId, bufferOut, function(writeInfo) {
                // we could handle this "nicely", but this will do for now
                // (another approach is however much more complicated):
                // we can setup an interval asking for data lets say every 200ms, when data arrives, callback will be triggered and tab switched
                // we could probably implement this someday
                GUI.timeout_add('waiting_for_bootup', function() {
                    CLI_active = false; 
                    
                    if (callback) callback();
                }, 5000); // if we dont allow enough time to reboot, CRC of "first" command sent will fail, keep an eye for this one
            });
            break;
        default:
            if (callback) callback();
    }
};

// initialize object into GUI variable
var GUI = new GUI_control();