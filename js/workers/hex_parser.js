'use strict';

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
        data:                   [],
        end_of_file:            false,
        bytes_total:            0,
        start_linear_address:   0
    };

    var extended_linear_address = 0;
    var next_address = 0;

    for (var i = 0; i < data.length && hexfile_valid; i++) {
        // each byte is represnted by two chars
        var byte_count = parseInt(data[i].substr(1, 2), 16);
        var address = parseInt(data[i].substr(3, 4), 16);
        var record_type = parseInt(data[i].substr(7, 2), 16);
        var content = data[i].substr(9, byte_count * 2); // still in string format
        var checksum = parseInt(data[i].substr(9 + byte_count * 2, 2), 16); // (this is a 2's complement value)

        switch (record_type) {
            case 0x00: // data record
                if (address != next_address || next_address == 0) {
                    result.data.push({'address': extended_linear_address + address, 'bytes': 0, 'data': []});
                }

                // store address for next comparison
                next_address = address + byte_count;

                // process data
                var crc = byte_count + parseInt(data[i].substr(3, 2), 16) + parseInt(data[i].substr(5, 2), 16) + record_type;
                for (var needle = 0; needle < byte_count * 2; needle += 2) { // * 2 because of 2 hex chars per 1 byte
                    var num = parseInt(content.substr(needle, 2), 16); // get one byte in hex and convert it to decimal
                    var data_block = result.data.length - 1;

                    result.data[data_block].data.push(num);
                    result.data[data_block].bytes++;

                    crc += num;
                    result.bytes_total++;
                }

                // change crc to 2's complement
                crc = (~crc + 1) & 0xFF;

                // verify
                if (crc != checksum) {
                    hexfile_valid = false;
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
                if (parseInt(content, 16) != 0) { // ignore if segment is 0
                    console.log('start segment address record found - NOT IMPLEMENTED !!!');
                }
                break;
            case 0x04: // extended linear address record
                extended_linear_address = (parseInt(content.substr(0, 2), 16) << 24) | parseInt(content.substr(2, 2), 16) << 16;
                break;
            case 0x05: // start linear address record
                result.start_linear_address = parseInt(content, 16)
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