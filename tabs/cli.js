'use strict';

var CLI_active = false;
var CLI_valid = false;

tabs.cli = {};
tabs.cli.initialize = function(callback) {
    var self = this;
    GUI.active_tab_ref = this;
    GUI.active_tab = 'cli';
    googleAnalytics.sendAppView('CLI Page');

    $('#content').load("./tabs/cli.html", function() {
        // translate to user-selected language
        localize();

        CLI_active = true;

        // Enter CLI mode
        var bufferOut = new ArrayBuffer(1);
        var bufView = new Uint8Array(bufferOut);

        bufView[0] = 0x23; // #

        serial.send(bufferOut, function(writeInfo) {});

        var textarea = $('.tab-cli textarea');

        textarea.keypress(function(event) {
            if (event.which == 13) { // enter
                event.preventDefault(); // prevent the adding of new line

                var out_string = textarea.val();
                var out_arr = out_string.split("\n");
                self.history.add(out_string.trim());

                var timeout_needle = 0;
                for (var i = 0; i < out_arr.length; i++) {
                    send_slowly(out_arr, i, timeout_needle++);
                }

                textarea.val('');
            }
        });

        textarea.keyup(function(event) {
            var keyUp = { 38: true }, keyDown = { 40: true };

            if (event.keyCode in keyUp)
                textarea.val(self.history.prev());

            if (event.keyCode in keyDown)
                textarea.val(self.history.next());
        });

        // give input element user focus
        textarea.focus();

        if (callback) callback();
    });
};

tabs.cli.history = {
    history: [],
    index:  0
};

tabs.cli.history.add = function(str) {
    this.history.push(str);
    this.index = this.history.length;
};
tabs.cli.history.prev = function() {
    if (this.index > 0) this.index -= 1;
    return this.history[this.index];
};
tabs.cli.history.next = function() {
    if (this.index < this.history.length) this.index += 1;
    return this.history[this.index - 1];
};

tabs.cli.cleanup = function(callback) {
    var bufferOut = new ArrayBuffer(5);
    var bufView = new Uint8Array(bufferOut);

    bufView[0] = 0x65; // e
    bufView[1] = 0x78; // x
    bufView[2] = 0x69; // i
    bufView[3] = 0x74; // t
    bufView[4] = 0x0D; // enter

    serial.send(bufferOut, function(writeInfo) {
        // we could handle this "nicely", but this will do for now
        // (another approach is however much more complicated):
        // we can setup an interval asking for data lets say every 200ms, when data arrives, callback will be triggered and tab switched
        // we could probably implement this someday
        GUI.timeout_add('waiting_for_bootup', function() {
            CLI_active = false;
            CLI_valid = false;

            if (callback) callback();
        }, 5000); // if we dont allow enough time to reboot, CRC of "first" command sent will fail, keep an eye for this one
    });
};

function send_slowly(out_arr, i, timeout_needle) {
    GUI.timeout_add('CLI_send_slowly', function() {
        var bufferOut = new ArrayBuffer(out_arr[i].length + 1);
        var bufView = new Uint8Array(bufferOut);

        for (var c_key = 0; c_key < out_arr[i].length; c_key++) {
            bufView[c_key] = out_arr[i].charCodeAt(c_key);
        }

        bufView[out_arr[i].length] = 0x0D; // enter (\n)

        serial.send(bufferOut, function(writeInfo) {});
    }, timeout_needle * 5);
}

/*  Some info about handling line feeds and carriage return

    line feed = LF = \n = 0x0A = 10
    carriage return = CR = \r = 0x0D = 13

    MAC only understands CR
    Linux and Unix only understand LF
    Windows understands (both) CRLF
    Chrome OS currenty unknown
*/

var sequence_elements = 0;
var CLI_validate_text = "";
function handle_CLI(readInfo) {
    var data = new Uint8Array(readInfo.data);
    var text = "";

    for (var i = 0; i < data.length; i++) {
        if (CLI_valid) {
            if (data[i] == 27 || sequence_elements > 0) { // ESC + other
                sequence_elements++;

                // delete previous space
                if (sequence_elements == 1) {
                    text = text.substring(0, text.length -1);
                }

                // Reset
                if (sequence_elements >= 5) {
                    sequence_elements = 0;
                }
            }

            if (sequence_elements == 0) {
                switch (data[i]) {
                    case 10: // line feed
                        if (GUI.operating_system != "MacOS") {
                            text += "<br />";
                        }
                        break;
                    case 13: // carriage return
                        if (GUI.operating_system == "MacOS") {
                            text += "<br />";
                        }
                        break;
                    default:
                        text += String.fromCharCode(data[i]);
                }
            }
        } else {
            // try to catch part of valid CLI enter message
            CLI_validate_text += String.fromCharCode(data[i]);
        }
    }

    if (!CLI_valid && CLI_validate_text.indexOf('CLI') != -1) {
        CLI_valid = true;
        CLI_validate_text = "";

        text = "Entering CLI Mode, type 'exit' to return, or 'help'<br /><br /># ";
    }

    $('.tab-cli .window .wrapper').append(text);
    $('.tab-cli .window').scrollTop($('.tab-cli .window .wrapper').height());

    // there seems to be some sort of initial rendering glitch in 33+, we will force redraw/refill
    $('.tab-cli .window .wrapper').css('webkitTransform', 'scale(1)');
}
