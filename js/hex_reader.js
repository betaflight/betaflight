// input = string
// result = if hex file is valid, result contains hex array of raw data (pure binary representation of firmware)
//          if hex file wasn't valid (crc check failed on any of the lines), resul will be false
function read_hex_file(data) {
    data = data.split("\n");
    
    // check if there is an empty line in the end of hex file, if there is, remove it
    if (data[data.length - 1] == "") {
        data.pop();
    }
    
    var raw_hex = new Array();
    var bytes_total = 0; // just for info / debug purposes
    var hexfile_valid = true; // if any of the crc checks failed, this variable flips to false
    for (var i = 0; i < data.length; i++) {
        var byte_count = parseInt(data[i].substr(1, 2), 16) * 2; // each byte is represnted by two chars (* 2 to get the hex representation)
        var address = data[i].substr(3, 4);
        var record_type = parseInt(data[i].substr(7, 2), 16); // also converting from hex to decimal
        var content = data[i].substr(9, byte_count);
        var checksum = parseInt(data[i].substr(9 + byte_count, 2), 16); // also converting from hex to decimal (this is a 2's complement value)
       
        if (record_type == 0x00) { // data record
            if (byte_count > 0) {
                var crc = (byte_count / 2) + parseInt(address.substr(0, 2), 16) + parseInt(address.substr(2, 2), 16) + record_type;
                for (var needle = 0; needle < byte_count; needle += 2) {
                    var num = parseInt(content.substr(needle, 2), 16); // get one byte in hex and convert it to decimal
                    raw_hex.push(num);
                    
                    crc += num;
                    bytes_total++;
                }
                
                // change crc to 2's complement (same as checksum)
                crc = ~crc + 1;
                crc &= 0xFF;
                
                // verify 
                if (crc != checksum) {
                    hexfile_valid = false;
                }
            }
        }
    }
    
    if (hexfile_valid) {
        console.log('HEX file parsed: ' + bytes_total + ' bytes');
        
        return raw_hex;
    } else {
        console.log('HEX file parsed, CRC check failed: ' + bytes_total + ' bytes');
        
        return false;
    }
}