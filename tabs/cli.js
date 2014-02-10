var CliHistory = function () {
    this.history = [];
    this.index = 0;
};

CliHistory.prototype = {
    add: function (str) {
        this.history.push(str);
        this.index = this.history.length;
    },
    prev: function () {
        if (this.index > 0) this.index -= 1;
        return this.history[this.index];
    },
    next: function () {
        if (this.index < this.history.length) this.index += 1;
        return this.history[this.index - 1];
    }
};

cli_history = new CliHistory();


function tab_initialize_cli() {
    ga_tracker.sendAppView('CLI Page');
    
    $('#content').load("./tabs/cli.html", function() {
        GUI.active_tab = 'cli';
        
        // remove any active interval for delayed command
        MSP.callbacks_cleanup();
        
        CLI_active = true;
        
        // Enter CLI mode
        var bufferOut = new ArrayBuffer(1);
        var bufView = new Uint8Array(bufferOut);
        
        bufView[0] = 0x23; // #

        serial.send(bufferOut, function(writeInfo) {});

        var textarea = $('.tab-cli textarea');
        textarea.keypress(function(event) {
            if (event.which == 13) { // enter
                var out_string = $('.tab-cli textarea').val();
                var out_arr = out_string.split("\n");
                cli_history.add(out_string.trim());
                var timeout_needle = 0;
                
                for (var i = 0; i < out_arr.length; i++) {
                    send_slowly(out_arr, i, timeout_needle++);
                }
                
                $('.tab-cli textarea').val('');
            }
        });

        textarea.keyup(function(event) {
            var keyUp = { 38: true }, keyDown = { 40: true };

            if (event.keyCode in keyUp)
                textarea.val(cli_history.prev());

            if (event.keyCode in keyDown)
                textarea.val(cli_history.next());
        });
        
        // handle smaller resolutions
        if (screen.height <= 600) {
            $('div.tab-cli .window').height(200);
        }
        
        // apply dynamic width to the textarea element according to cli window width (minus padding and border width)
        $('div.tab-cli textarea').width($('div.tab-cli .window').outerWidth() - 7);
        
        // give input element user focus
        $('.tab-cli textarea').focus();
        
        $('.tab-cli .copy').click(function() {
            var text = $('.tab-cli .window .wrapper').html();
            text = text.replace(/<br\s*\/?>/mg,"\n"); // replacing br tags with \n to keep some of the formating
            
            var copyFrom = $('<textarea/>');
            
            copyFrom.text(text);
            $('body').append(copyFrom);
            copyFrom.select();
            document.execCommand('copy');
            copyFrom.remove();
        });
    });
}

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
                        if (GUI.operating_system == "Windows" || GUI.operating_system == "Linux" || GUI.operating_system == "UNIX") {
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
        
        char_counter++;
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
