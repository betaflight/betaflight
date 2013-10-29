function tab_initialize_auxiliary_configuration() {
    ga_tracker.sendAppView('Auxiliary Configuration');
    
    // generate table from the supplied AUX names and AUX data
    for (var i = 0; i < AUX_CONFIG.length; i++) {
        $('.tab-auxiliary_configuration .boxes > tbody:last').append(
            '<tr>' +
                '<td class="name">' + AUX_CONFIG[i] + '</td>' +
                box_check(AUX_CONFIG_values[i], 0) +
                box_check(AUX_CONFIG_values[i], 1) +
                box_check(AUX_CONFIG_values[i], 2) +
                
                box_check(AUX_CONFIG_values[i], 3) +
                box_check(AUX_CONFIG_values[i], 4) +
                box_check(AUX_CONFIG_values[i], 5) +

                box_check(AUX_CONFIG_values[i], 6) +
                box_check(AUX_CONFIG_values[i], 7) +
                box_check(AUX_CONFIG_values[i], 8) +

                box_check(AUX_CONFIG_values[i], 9) +
                box_check(AUX_CONFIG_values[i], 10) +
                box_check(AUX_CONFIG_values[i], 11) +            
            '</tr>'
        );
    }
    
    // UI Hooks
    $('.tab-auxiliary_configuration .boxes input').change(function() {
        // if any of the fields changed, unlock update button
        $('a.update').addClass('active');
    });
    
    $('.tab-auxiliary_configuration a.update').click(function() {
        if ($(this).hasClass('active')) {
            // catch the input changes
            var main_needle = 0;
            var needle = 0;
            $('.tab-auxiliary_configuration .boxes input').each(function() {
                if ($(this).is(':checked')) {
                    AUX_CONFIG_values[main_needle] = bit_set(AUX_CONFIG_values[main_needle], needle);
                } else {
                    AUX_CONFIG_values[main_needle] = bit_clear(AUX_CONFIG_values[main_needle], needle);
                }
                
                needle++;
                
                if (needle >= 12) { // 4 aux * 3 checkboxes = 12 bits per line
                    main_needle++;
                    
                    needle = 0;
                }
            });
            
            // send over the data
            var AUX_val_buffer_out = new Array();
            
            var needle = 0;
            for (var i = 0; i < AUX_CONFIG_values.length; i++) {
                AUX_val_buffer_out[needle++] = lowByte(AUX_CONFIG_values[i]);
                AUX_val_buffer_out[needle++] = highByte(AUX_CONFIG_values[i]);
            }
            
            send_message(MSP_codes.MSP_SET_BOX, AUX_val_buffer_out); 

            // Save changes to EEPROM
            send_message(MSP_codes.MSP_EEPROM_WRITE, MSP_codes.MSP_EEPROM_WRITE);
            
            // remove the active status
            $(this).removeClass('active');
        }
    });

    // enable data pulling
    timers.push(setInterval(aux_data_poll, 50));       
}

function aux_data_poll() {
    send_message(MSP_codes.MSP_STATUS, MSP_codes.MSP_STATUS);
    send_message(MSP_codes.MSP_RC, MSP_codes.MSP_RC);
    
    for (var i = 0; i < AUX_CONFIG.length; i++) {
        if (bit_check(CONFIG.mode, i)) {
            $('td.name').eq(i).addClass('on').removeClass('off'); 
        } else {
            $('td.name').eq(i).removeClass('on').removeClass('off');
            
            if (AUX_CONFIG_values[i] > 0) {
                $('td.name').eq(i).addClass('off');
            }
        }
    }
    
    box_highlight(RC.AUX1, 2);
    box_highlight(RC.AUX2, 5);
    box_highlight(RC.AUX3, 8);
    box_highlight(RC.AUX4, 11);
}

function box_check(num, pos) {
    if (bit_check(num, pos)) { // 1
        return '<td><input type="checkbox" checked="checked" /></td>';
    } else { // 0
        return '<td><input type="checkbox" /></td>';
    }
}

// val = channel value
// aux_num = position of corresponding aux channel in the html table
function box_highlight(val, aux_num) {
    var tr = $('table.boxes tr');
    var pos = 0; // < 1300
    
    if (val > 1300 && val < 1700) {
        pos = 1;
    } else if (val > 1700) {
        pos = 2;
    }

    $(':nth-child(' + aux_num + '), :nth-child(' + (aux_num + 1) + '), :nth-child(' + (aux_num + 2) + ')', tr).css('background-color', 'transparent');
    $('td:nth-child(' + (aux_num + pos) + ')', tr).css('background-color', 'orange');
}