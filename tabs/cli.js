function tab_initialize_cli() {
    CLI_active = true;
    
    // Enter CLI mode
    var bufferOut = new ArrayBuffer(1);
    var bufView = new Uint8Array(bufferOut);
    
    bufView[0] = 0x23; // #

    chrome.serial.write(connectionId, bufferOut, function(writeInfo) {
        // used for debugging purposes (should be disabled in "stable" builds
        console.log("Wrote: " + writeInfo.bytesWritten + " bytes");
    });

    $('.tab-cli input').keypress(function(event) {
        if (event.which == 13) { // enter
            var out_string = $('.tab-cli input').val();
            
            var bufferOut = new ArrayBuffer(out_string.length + 1); // +1 for enter character
            var bufView = new Uint8Array(bufferOut);
            
            for (var i = 0; i < out_string.length; i++) {
                bufView[i] = out_string.charCodeAt(i);
            }
            
            bufView[out_string.length] = 0x0D; // enter
            
            chrome.serial.write(connectionId, bufferOut, function(writeInfo) {
                $('.tab-cli input').val('');
            });
        }
    });
    
    // give input element user focus
    $('.tab-cli input').focus();
}

function leave_CLI(callback) {
    var bufferOut = new ArrayBuffer(5);
    var bufView = new Uint8Array(bufferOut);
    
    bufView[0] = 0x65; // e
    bufView[1] = 0x78; // x
    bufView[2] = 0x69; // i
    bufView[3] = 0x74; // t
    bufView[4] = 0x0D; // enter

    chrome.serial.write(connectionId, bufferOut, function(writeInfo) {
        if (typeof callback !== 'undefined') {
            callback();
        }
    });   

    CLI_active = false;    
}


/*  Some info about handling line feeds and carriage return

    line feed = LF = \n = 0x0A = 10
    carriage return = CR = \r = 0x0D = 13
    
    MAC only understands CR
    Linux and Unix only understand LF
    Windows understands (both) CRLF
*/
function handle_CLI(data) {
    switch (data) {
        case 10: // line feed
            if (OS == "Windows" || OS == "Linux" || OS == "UNIX") {
                $('.tab-cli .window .wrapper').append("<br />");
            }
            break;
        case 13: // carriage return
            if (OS == "MacOS") {
                $('.tab-cli .window .wrapper').append("<br />");
            }
            break;
        default:
            $('.tab-cli .window .wrapper').append(String.fromCharCode(data));
            $('.tab-cli .window').scrollTop($('.tab-cli .window .wrapper').height());
    }
}