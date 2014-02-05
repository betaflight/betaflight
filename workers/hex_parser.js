// input = string
// result = if hex file is valid, result is an object
//          if hex file wasn't valid (crc check failed on any of the lines), result will be false
function read_hex_file(data) {
    data = data.split("\n");
    
    // check if there is an empty line in the end of hex file, if there is, remove it
    if (data[data.length - 1] == "") {
        data.pop();
    }
    
    var hexfile_valid = true; // if any of the crc checks failed, this variable flips to false
    
    var result = {
        data:                       [],
        end_of_file:                false,
        extended_linear_address:    [],
        start_linear_address:       0,
        bytes:                      0
    };
    
    var next_address_pos = 0;
    
    for (var i = 0; i < data.length; i++) {
        var byte_count = parseInt(data[i].substr(1, 2), 16); // each byte is represnted by two chars
        var address = data[i].substr(3, 4);
        var record_type = parseInt(data[i].substr(7, 2), 16); // also converting from hex to decimal
        var content = data[i].substr(9, byte_count * 2);
        var checksum = parseInt(data[i].substr(9 + byte_count * 2, 2), 16); // also converting from hex to decimal (this is a 2's complement value)
       
        switch (record_type) {
            case 0x00: // data record
                // fix "holes" if there are any
                if (parseInt(address, 16) != next_address_pos) {
                    var difference = parseInt(address, 16) - (next_address_pos);
                    
                    // fill in the difference
                    for (var x = 0; x < difference; x++) {
                        result.data.push(0xFF);
                        result.bytes++;
                    }
                    
                    console.log('HEX_PARSER - Address hole detected, expected: ' + next_address_pos + ', received: ' + parseInt(address, 16) + ', filling: ' + difference + ' bytes');
                }
                
                // update for next comparison
                next_address_pos = parseInt(address, 16) + byte_count;
                
                // process data
                var crc = byte_count + parseInt(address.substr(0, 2), 16) + parseInt(address.substr(2, 2), 16) + record_type;
                for (var needle = 0; needle < byte_count * 2; needle += 2) {
                    var num = parseInt(content.substr(needle, 2), 16); // get one byte in hex and convert it to decimal
                    result.data.push(num);
                    
                    crc += num;
                    result.bytes++;
                }
                
                // change crc to 2's complement (same as checksum)
                crc = ~crc + 1;
                crc &= 0xFF;
                
                // verify 
                if (crc != checksum) {
                    hexfile_valid = false;
                    
                    // break out of the for loop as crc is wrong anyway, we dont need to process any more data
                    i = data.length;
                }
                break;
            case 0x01: // end of file record
                result.end_of_file = true;
                break;
            case 0x02: // extended segment address record
                // not implemented
                if (parseInt(content, 16) != 0) { // ignore if segment is 0
                    console.log('extended segment address record found - NOT IMPLEMENTED !!!');
                }
                break;
            case 0x03: // start segment address record
                // not implemented
                console.log('start segment address record found - NOT IMPLEMENTED !!!');
                break;
            case 0x04: // extended linear address record                
                var extended_linear_address = (parseInt(content.substr(0, 2), 16) << 24) | parseInt(content.substr(2, 2), 16) << 16;
                result.extended_linear_address.push(extended_linear_address);
                
                if (next_address_pos != 0) { // dont execute the first time
                    extended_linear_address -= 0x08000000;
                    var difference = extended_linear_address - next_address_pos;
                    
                    // fill in the difference
                    for (var x = 0; x < difference; x++) {
                        result.data.push(0xFF);
                        result.bytes++;
                    }
                    
                    if (difference > 0) console.log('HEX_PARSER - Address hole detected (changing linear address), expected: ' + next_address_pos + ', received: ' + extended_linear_address + ', filling: ' + difference + ' bytes');
                    
                    // reset some variables
                    next_address_pos = 0;
                }
                break;
            case 0x05: // start linear address record
                result.start_linear_address = (parseInt(content.substr(0, 2), 16) << 24) | (parseInt(content.substr(2, 2), 16) << 16) | (parseInt(content.substr(4, 2), 16) << 8) | parseInt(content.substr(6, 2), 16);
                break;
        }
    }
    
    if (result.end_of_file && hexfile_valid) {
        postMessage(result);
    } else {
        postMessage(false);
    }
}

function microtime() {
    var now = new Date().getTime() / 1000;

    return now;
}

onmessage = function(event) {
    var time_parsing_start = microtime(); // track time
    
    read_hex_file(event.data);
    
    console.log('HEX_PARSER - File parsed in: ' + (microtime() - time_parsing_start).toFixed(4) + ' seconds');
    
    // terminate worker
    close();
};