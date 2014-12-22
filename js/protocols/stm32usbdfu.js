/*
    USB DFU uses:
    control transfers for communicating
    recipient is interface
    request type is class

    Descriptors seems to be broken in current chrome.usb API implementation (writing this while using canary 37.0.2040.0

    General rule to remember is that DFU doesn't like running specific operations while the device isn't in idle state
    that being said, it seems that certain level of CLRSTATUS is required before running another type of operation for
    example switching from DNLOAD to UPLOAD, etc, clearning the state so device is in dfuIDLE is highly recommended.
*/
'use strict';

var STM32DFU_protocol = function () {
    this.callback; // ref
    this.hex; // ref
    this.verify_hex;

    this.handle = null; // connection handle

    this.request = {
        DETACH:     0x00, // OUT, Requests the device to leave DFU mode and enter the application.
        DNLOAD:     0x01, // OUT, Requests data transfer from Host to the device in order to load them into device internal Flash. Includes also erase commands
        UPLOAD:     0x02, // IN,  Requests data transfer from device to Host in order to load content of device internal Flash into a Host file.
        GETSTATUS:  0x03, // IN,  Requests device to send status report to the Host (including status resulting from the last request execution and the state the device will enter immediately after this request).
        CLRSTATUS:  0x04, // OUT, Requests device to clear error status and move to next step
        GETSTATE:   0x05, // IN,  Requests the device to send only the state it will enter immediately after this request.
        ABORT:      0x06  // OUT, Requests device to exit the current state/operation and enter idle state immediately.
    };

    this.status = {
        OK:                 0x00, // No error condition is present.
        errTARGET:          0x01, // File is not targeted for use by this device.
        errFILE:            0x02, // File is for this device but fails some vendor-specific verification test
        errWRITE:           0x03, // Device is unable to write memory.
        errERASE:           0x04, // Memory erase function failed.
        errCHECK_ERASED:    0x05, // Memory erase check failed.
        errPROG:            0x06, // Program memory function failed.
        errVERIFY:          0x07, // Programmed memory failed verification.
        errADDRESS:         0x08, // Cannot program memory due to received address that is out of range.
        errNOTDONE:         0x09, // Received DFU_DNLOAD with wLength = 0, but device does not think it has all of the data yet.
        errFIRMWARE:        0x0A, // Device's firmware is corrupt. It cannot return to run-time (non-DFU) operations.
        errVENDOR:          0x0B, // iString indicates a vendor-specific error.
        errUSBR:            0x0C, // Device detected unexpected USB reset signaling.
        errPOR:             0x0D, // Device detected unexpected power on reset.
        errUNKNOWN:         0x0E, // Something went wrong, but the device does not know what it was.
        errSTALLEDPKT:      0x0F  // Device stalled an unexpected request.
    };

    this.state = {
        appIDLE:                0, // Device is running its normal application.
        appDETACH:              1, // Device is running its normal application, has received the DFU_DETACH request, and is waiting for a USB reset.
        dfuIDLE:                2, // Device is operating in the DFU mode and is waiting for requests.
        dfuDNLOAD_SYNC:         3, // Device has received a block and is waiting for the host to solicit the status via DFU_GETSTATUS.
        dfuDNBUSY:              4, // Device is programming a control-write block into its nonvolatile memories.
        dfuDNLOAD_IDLE:         5, // Device is processing a download operation. Expecting DFU_DNLOAD requests.
        dfuMANIFEST_SYNC:       6, // Device has received the final block of firmware from the host and is waiting for receipt of DFU_GETSTATUS to begin the Manifestation phase; or device has completed the Manifestation phase and is waiting for receipt of DFU_GETSTATUS.
        dfuMANIFEST:            7, // Device is in the Manifestation phase. (Not all devices will be able to respond to DFU_GETSTATUS when in this state.)
        dfuMANIFEST_WAIT_RESET: 8, // Device has programmed its memories and is waiting for a USB reset or a power on reset. (Devices that must enter this state clear bitManifestationTolerant to 0.)
        dfuUPLOAD_IDLE:         9, // The device is processing an upload operation. Expecting DFU_UPLOAD requests.
        dfuERROR:               10 // An error has occurred. Awaiting the DFU_CLRSTATUS request.
    };
};

STM32DFU_protocol.prototype.connect = function (device, hex, callback) {
    var self = this;
    self.hex = hex;
    self.callback = callback;

    // reset and set some variables before we start
    self.upload_time_start = new Date().getTime();
    self.verify_hex = [];

    // reset progress bar to initial state
    self.progress_bar_e = $('.progress');
    self.progress_bar_e.val(0);
    self.progress_bar_e.removeClass('valid invalid');

    chrome.usb.getDevices(device, function (result) {
        if (result.length) {
            console.log('USB DFU detected with ID: ' + result[0].device);

            self.openDevice(result[0]);
        } else {
            console.log('USB DFU not found');
            GUI.log('USB DFU not found');
        }
    });
};

STM32DFU_protocol.prototype.openDevice = function (device) {
    var self = this;

    chrome.usb.openDevice(device, function (handle) {
        self.handle = handle;

        console.log('Device opened with Handle ID: ' + handle.handle);
        self.claimInterface(0);
    });
};

STM32DFU_protocol.prototype.closeDevice = function () {
    var self = this;

    chrome.usb.closeDevice(this.handle, function closed() {
        console.log('Device closed with Handle ID: ' + self.handle.handle);

        self.handle = null;
    });
};

STM32DFU_protocol.prototype.claimInterface = function (interfaceNumber) {
    var self = this;

    chrome.usb.claimInterface(this.handle, interfaceNumber, function claimed() {
        console.log('Claimed interface: ' + interfaceNumber);

        self.upload_procedure(1);
    });
};

STM32DFU_protocol.prototype.releaseInterface = function (interfaceNumber) {
    var self = this;

    chrome.usb.releaseInterface(this.handle, interfaceNumber, function released() {
        console.log('Released interface: ' + interfaceNumber);

        self.closeDevice();
    });
};

STM32DFU_protocol.prototype.resetDevice = function (callback) {
    chrome.usb.resetDevice(this.handle, function (result) {
        console.log('Reset Device: ' + result);

        if (callback) callback();
    });
};

STM32DFU_protocol.prototype.controlTransfer = function (direction, request, value, _interface, length, data, callback) {
    if (direction == 'in') {
        // data is ignored
        chrome.usb.controlTransfer(this.handle, {
            'direction':    'in',
            'recipient':    'interface',
            'requestType':  'class',
            'request':      request,
            'value':        value,
            'index':        _interface,
            'length':       length
        }, function (result) {
            if (result.resultCode) console.log(result.resultCode);

            var buf = new Uint8Array(result.data);
            callback(buf, result.resultCode);
        });
    } else {
        // length is ignored
        if (data) {
            var arrayBuf = new ArrayBuffer(data.length);
            var arrayBufView = new Uint8Array(arrayBuf);
            arrayBufView.set(data);
        } else {
            var arrayBuf = new ArrayBuffer(0);
        }

        chrome.usb.controlTransfer(this.handle, {
            'direction':    'out',
            'recipient':    'interface',
            'requestType':  'class',
            'request':      request,
            'value':        value,
            'index':        _interface,
            'data':         arrayBuf
        }, function (result) {
            if (result.resultCode) console.log(result.resultCode);

            callback(result);
        });
    }
};

// routine calling DFU_CLRSTATUS until device is in dfuIDLE state
STM32DFU_protocol.prototype.clearStatus = function (callback) {
    var self = this;

    function check_status() {
        self.controlTransfer('in', self.request.GETSTATUS, 0, 0, 6, 0, function (data) {
            if (data[4] == self.state.dfuIDLE) {
                callback(data);
            } else {
                var delay = data[1] | (data[2] << 8) | (data[3] << 16);

                setTimeout(clear_status, delay);
            }
        });
    }

    function clear_status() {
        self.controlTransfer('out', self.request.CLRSTATUS, 0, 0, 0, 0, check_status);
    }

    check_status();
};

STM32DFU_protocol.prototype.loadAddress = function (address, callback) {
    var self = this;

    self.controlTransfer('out', self.request.DNLOAD, 0, 0, 0, [0x21, address, (address >> 8), (address >> 16), (address >> 24)], function () {
        self.controlTransfer('in', self.request.GETSTATUS, 0, 0, 6, 0, function (data) {
            if (data[4] == self.state.dfuDNBUSY) {
                var delay = data[1] | (data[2] << 8) | (data[3] << 16);

                setTimeout(function () {
                    self.controlTransfer('in', self.request.GETSTATUS, 0, 0, 6, 0, function (data) {
                        if (data[4] == self.state.dfuDNLOAD_IDLE) {
                            callback(data);
                        } else {
                            console.log('Failed to execure address load');
                            self.upload_procedure(99);
                        }
                    });
                }, delay);
            } else {
                console.log('Failed to request address load');
                self.upload_procedure(99);
            }
        });
    });
};

// first_array = usually hex_to_flash array
// second_array = usually verify_hex array
// result = true/false
STM32DFU_protocol.prototype.verify_flash = function (first_array, second_array) {
    for (var i = 0; i < first_array.length; i++) {
        if (first_array[i] != second_array[i]) {
            console.log('Verification failed on byte: ' + i + ' expected: 0x' + first_array[i].toString(16) + ' received: 0x' + second_array[i].toString(16));
            return false;
        }
    }

    console.log('Verification successful, matching: ' + first_array.length + ' bytes');

    return true;
};

STM32DFU_protocol.prototype.upload_procedure = function (step) {
    var self = this;

    switch (step) {
        case 1:
            self.clearStatus(function () {
                self.upload_procedure(2);
            });
            break;
        case 2:
            // full chip erase
            console.log('Executing global chip erase');
            $('span.progressLabel').text('Erasing ...');

            self.controlTransfer('out', self.request.DNLOAD, 0, 0, 0, [0x41], function () {
                self.controlTransfer('in', self.request.GETSTATUS, 0, 0, 6, 0, function (data) {
                    if (data[4] == self.state.dfuDNBUSY) { // completely normal
                        var delay = data[1] | (data[2] << 8) | (data[3] << 16);

                        setTimeout(function () {
                            self.controlTransfer('in', self.request.GETSTATUS, 0, 0, 6, 0, function (data) {
                                if (data[4] == self.state.dfuDNLOAD_IDLE) {
                                    self.upload_procedure(4);
                                } else {
                                    console.log('Failed to execute global chip erase');
                                    self.upload_procedure(99);
                                }
                            });
                        }, delay);
                    } else {
                        console.log('Failed to initiate global chip erase');
                        self.upload_procedure(99);
                    }
                });
            });
            break;
        case 4:
            // upload
            // we dont need to clear the state as we are already using DFU_DNLOAD
            console.log('Writing data ...');
            $('span.progressLabel').text('Flashing ...');

            var blocks = self.hex.data.length - 1;
            var flashing_block = 0;
            var address = self.hex.data[flashing_block].address;

            var bytes_flashed = 0;
            var bytes_flashed_total = 0; // used for progress bar
            var wBlockNum = 2; // required by DFU

            // start
            self.loadAddress(address, write);

            var write = function () {
                if (bytes_flashed < self.hex.data[flashing_block].bytes) {
                    var bytes_to_write = ((bytes_flashed + 2048) <= self.hex.data[flashing_block].bytes) ? 2048 : (self.hex.data[flashing_block].bytes - bytes_flashed);

                    var data_to_flash = self.hex.data[flashing_block].data.slice(bytes_flashed, bytes_flashed + bytes_to_write);

                    address += bytes_to_write;
                    bytes_flashed += bytes_to_write;
                    bytes_flashed_total += bytes_to_write;

                    self.controlTransfer('out', self.request.DNLOAD, wBlockNum++, 0, 0, data_to_flash, function () {
                        self.controlTransfer('in', self.request.GETSTATUS, 0, 0, 6, 0, function (data) {
                            if (data[4] == self.state.dfuDNBUSY) {
                                var delay = data[1] | (data[2] << 8) | (data[3] << 16);

                                setTimeout(function () {
                                    self.controlTransfer('in', self.request.GETSTATUS, 0, 0, 6, 0, function (data) {
                                        if (data[4] == self.state.dfuDNLOAD_IDLE) {
                                            // update progress bar
                                            self.progress_bar_e.val(bytes_flashed_total / (self.hex.bytes_total * 2) * 100);

                                            // flash another page
                                            write();
                                        } else {
                                            console.log('Failed to write ' + bytes_to_write + 'bytes to 0x' + address.toString(16));
                                            self.upload_procedure(99);
                                        }
                                    });
                                }, delay);
                            } else {
                                console.log('Failed to initiate write ' + bytes_to_write + 'bytes to 0x' + address.toString(16));
                                self.upload_procedure(99);
                            }
                        });
                    })
                } else {
                    if (flashing_block < blocks) {
                        // move to another block
                        flashing_block++;

                        address = self.hex.data[flashing_block].address;
                        bytes_flashed = 0;
                        wBlockNum = 2;

                        self.loadAddress(address, write);
                    } else {
                        // all blocks flashed
                        console.log('Writing: done');

                        // proceed to next step
                        self.upload_procedure(5);
                    }
                }
            }
            break;
        case 5:
            // verify
            console.log('Verifying data ...');
            $('span.progressLabel').text('Verifying ...');

            var blocks = self.hex.data.length - 1;
            var reading_block = 0;
            var address = self.hex.data[reading_block].address;

            var bytes_verified = 0;
            var bytes_verified_total = 0; // used for progress bar
            var wBlockNum = 2; // required by DFU

            // initialize arrays
            for (var i = 0; i <= blocks; i++) {
                self.verify_hex.push([]);
            }

            // start
            self.clearStatus(function () {
                self.loadAddress(address, function () {
                    self.clearStatus(read);
                });
            });

            var read = function () {
                if (bytes_verified < self.hex.data[reading_block].bytes) {
                    var bytes_to_read = ((bytes_verified + 2048) <= self.hex.data[reading_block].bytes) ? 2048 : (self.hex.data[reading_block].bytes - bytes_verified);

                    self.controlTransfer('in', self.request.UPLOAD, wBlockNum++, 0, bytes_to_read, 0, function (data, code) {
                        for (var i = 0; i < data.length; i++) {
                            self.verify_hex[reading_block].push(data[i]);
                        }

                        address += bytes_to_read;
                        bytes_verified += bytes_to_read;
                        bytes_verified_total += bytes_to_read;

                        // update progress bar
                        self.progress_bar_e.val((self.hex.bytes_total + bytes_verified_total) / (self.hex.bytes_total * 2) * 100);

                        // verify another page
                        read();
                    });
                } else {
                    if (reading_block < blocks) {
                        // move to another block
                        reading_block++;

                        address = self.hex.data[reading_block].address;
                        bytes_verified = 0;
                        wBlockNum = 2;

                        self.clearStatus(function () {
                            self.loadAddress(address, function () {
                                self.clearStatus(read);
                            });
                        });
                    } else {
                        // all blocks read, verify

                        var verify = true;
                        for (var i = 0; i <= blocks; i++) {
                            verify = self.verify_flash(self.hex.data[i].data, self.verify_hex[i]);

                            if (!verify) break;
                        }

                        if (verify) {
                            console.log('Programming: SUCCESSFUL');
                            $('span.progressLabel').text('Programming: SUCCESSFUL');
                            googleAnalytics.sendEvent('Flashing', 'Programming', 'success');

                            // update progress bar
                            self.progress_bar_e.addClass('valid');

                            // proceed to next step
                            self.upload_procedure(6);
                        } else {
                            console.log('Programming: FAILED');
                            $('span.progressLabel').text('Programming: FAILED');
                            googleAnalytics.sendEvent('Flashing', 'Programming', 'fail');

                            // update progress bar
                            self.progress_bar_e.addClass('invalid');

                            // disconnect
                            self.upload_procedure(99);
                        }
                    }
                }
            }
            break;
        case 6:
            // jump to application code
            var address = self.hex.data[0].address;

            self.clearStatus(function () {
                self.loadAddress(address, leave);
            });

            var leave = function () {
                self.controlTransfer('out', self.request.DNLOAD, 0, 0, 0, 0, function () {
                    self.controlTransfer('in', self.request.GETSTATUS, 0, 0, 6, 0, function (data) {
                        self.upload_procedure(99);
                    });
                });
            }

            // start
            clear_before_leave();
            break;
        case 99:
            // cleanup
            self.releaseInterface(0);

            var timeSpent = new Date().getTime() - self.upload_time_start;

            console.log('Script finished after: ' + (timeSpent / 1000) + ' seconds');

            if (self.callback) self.callback();
            break;
    }
};

// initialize object
var STM32DFU = new STM32DFU_protocol();