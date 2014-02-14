/*  Please don't take code in this file very seriously !!!

    I was "kinda" forced to write this implementation "this way" because the Servo code implementation
    from multiwii is so horrible, obstructive and non dynamic, not to mention it doesn't make any sense
    that there was just no other way around this then hardcoding/implementing each model separately.
*/

function tab_initialize_servos() {
    ga_tracker.sendAppView('Servos');
    GUI.active_tab = 'servos';
    
    // request current Servos Config
    send_message(MSP_codes.MSP_IDENT, MSP_codes.MSP_IDENT, false, function() {
        send_message(MSP_codes.MSP_SERVO_CONF, MSP_codes.MSP_SERVO_CONF, false, function() {
            send_message(MSP_codes.MSP_BOXNAMES, MSP_codes.MSP_BOXNAMES, false, function() {
                $('#content').load("./tabs/servos.html", function() {
                    // drop previous table
                    $('div.tab-servos table.fields tr:not(:first)').remove();
                    
                    var model = $('div.tab-servos strong.model');
                    var supported_models = [1, 4, 5, 8, 14, 20, 21];
                    
                    switch (CONFIG.multiType) {
                        case 1: // TRI
                            // looking ok so far
                            model.html('TRI');
                            
                            process_directions('YAW', 5, 0);
                            
                            process_servos('Yaw Servo', '', 5, false);
                            break;
                        case 4: // BI
                            // looking ok so far
                            model.html('BI');
                            
                            process_directions('L YAW', 4, 1);
                            process_directions('R YAW', 5, 1);
                            process_directions('L NICK', 4, 0);
                            process_directions('R NICK', 5, 0);
                            
                            process_servos('Left Servo', '', 4, false);
                            process_servos('Right Servo', '', 5, false);
                            break;
                        case 5: // Gimbal
                            // needs to be verified
                            model.html('Gimbal');
                            
                            // rate
                            process_servos('Pitch Servo', '', 0, 2);
                            process_servos('Roll Servo', '', 1, 2);
                            break;
                        case 8: // Flying Wing
                            // looking ok so far
                            model.html('Flying Wing');
                            
                            process_directions('L ROLL', 3, 1);
                            process_directions('R ROLL', 4, 1);
                            process_directions('L NICK', 3, 0);
                            process_directions('R NICK', 4, 0);
                            
                            process_servos('Left Wing', '', 3, false);
                            process_servos('Right Wing', '', 4, false);
                            break;
                        case 14: // Airplane
                            model.html('Airplane');
                            
                            // rate
                            process_servos('Wing 1', '', 3, 2);
                            process_servos('Wing 2', '', 4, 2);
                            process_servos('Rudd', '', 5, 2);
                            process_servos('Elev', '', 6, 2);
                            break;
                        case 20: // Dualcopter
                            // looking ok so far
                            model.html('Dualcopter');
                            
                            process_directions('PITCH', 4, 0);
                            process_directions('ROLL', 5, 0);
                            
                            process_servos('Roll', '', 5, false);
                            process_servos('Nick', '', 4, false);
                            break;
                        case 21: // Singlecopter
                            // looking ok so far
                            model.html('Singlecopter');
                            
                            process_servos('Right', 'R YAW', 3, true);
                            process_servos('Left', 'L YAW', 4, true);
                            process_servos('Front', 'F YAW', 5, true);
                            process_servos('Rear', 'YAW', 6, true);
                            break;
                            
                        default:
                            model.html("This model doesn't support servos");
                            
                            // implementation of feature servo_tilt
                            if (AUX_CONFIG.indexOf('CAMSTAB') > -1 || AUX_CONFIG.indexOf('CAMTRIG') > -1) {
                                // Gimbal on
                                // needs to be verified
                                model.html('Gimbal / Tilt Servos');
                                
                                // rate
                                process_servos('Pitch Servo', '', 0, 2);
                                process_servos('Roll Servo', '', 1, 2);
                            }
                    }
                    
                    // UI hooks for dynamically generated elements
                    $('table.directions select, table.directions input, table.fields select, table.fields input').change(function() {
                        if ($('div.live input').is(':checked')) {
                            // apply small delay as there seems to be some funky update business going wrong
                            GUI.timeout_add('servos_update', servos_update, 10);
                        }
                    });
                    
                    $('a.update').click(function() {
                        // standard check for supported_models + custom implementation for feature servo_tilt
                        if (supported_models.indexOf(CONFIG.multiType) != -1 || AUX_CONFIG.indexOf('CAMSTAB') > -1 || AUX_CONFIG.indexOf('CAMTRIG') > -1) {
                            servos_update(true);
                        }
                    });
                    
                    // enable data pulling
                    GUI.interval_add('servos_data_poll', function() {
                        send_message(MSP_codes.MSP_STATUS, MSP_codes.MSP_STATUS);
                    }, 50);
                });
            });
        });
    });
}

function servos_update(save_to_eeprom) {
    // update bitfields
    $('div.tab-servos table.directions tr:not(".main")').each(function() {
        var info = $('select', this).data('info');
        var val = parseInt($('select', this).val());
        
        // in this stage we need to know which bitfield and which bitposition needs to be flipped
        if (val) SERVO_CONFIG[info.obj].rate = bit_set(SERVO_CONFIG[info.obj].rate, info.bitpos);
        else SERVO_CONFIG[info.obj].rate = bit_clear(SERVO_CONFIG[info.obj].rate, info.bitpos);
    });
    
    // update the rest
    $('div.tab-servos table.fields tr:not(".main")').each(function() {
        var info = $(this).data('info');
        
        if ($('.middle input', this).is(':disabled')) {
            var val = $('.channel input:checked', this).index();
            
            SERVO_CONFIG[info.obj].middle = parseInt(val);
        } else {
            SERVO_CONFIG[info.obj].middle = parseInt($('.middle input', this).val());
        }
        
        SERVO_CONFIG[info.obj].min = parseInt($('.min input', this).val());
        SERVO_CONFIG[info.obj].max = parseInt($('.max input', this).val());

        // update rate if direction fields exist
        if ($('.direction input', this).length) {
            if ($('.direction input:first', this).is(':checked')) SERVO_CONFIG[info.obj].rate = bit_set(SERVO_CONFIG[info.obj].rate, 0);
            else SERVO_CONFIG[info.obj].rate = bit_clear(SERVO_CONFIG[info.obj].rate, 0);
            
            if ($('.direction input:last', this).is(':checked')) SERVO_CONFIG[info.obj].rate = bit_set(SERVO_CONFIG[info.obj].rate, 1);
            else SERVO_CONFIG[info.obj].rate = bit_clear(SERVO_CONFIG[info.obj].rate, 1);
        } else if ($('.direction select', this).length) {
            var val = parseInt($('.direction select', this).val());
            SERVO_CONFIG[info.obj].rate = val;                
        }
    });
    
    // send settings over to mcu
    var buffer_out = [];
    
    var needle = 0;
    for (var i = 0; i < SERVO_CONFIG.length; i++) {
        buffer_out[needle++] = lowByte(SERVO_CONFIG[i].min);
        buffer_out[needle++] = highByte(SERVO_CONFIG[i].min);
        
        buffer_out[needle++] = lowByte(SERVO_CONFIG[i].max);
        buffer_out[needle++] = highByte(SERVO_CONFIG[i].max);
        
        buffer_out[needle++] = lowByte(SERVO_CONFIG[i].middle);
        buffer_out[needle++] = highByte(SERVO_CONFIG[i].middle);
        
        buffer_out[needle++] = lowByte(SERVO_CONFIG[i].rate);
    }
    
    send_message(MSP_codes.MSP_SET_SERVO_CONF, buffer_out);
    
    if (save_to_eeprom) {
        // Save changes to EEPROM
        send_message(MSP_codes.MSP_EEPROM_WRITE, MSP_codes.MSP_EEPROM_WRITE, false, function() {
            GUI.log('EEPROM <span style="color: green">saved</span>');
            
            var element = $('a.update');
            element.addClass('success');
            
            GUI.timeout_add('success_highlight', function() {
                element.removeClass('success');
            }, 2000);
        });
    }
}

function process_directions(name, obj, bitpos) {
    $('div.direction_wrapper').show();
    
    var val;
    
    $('div.tab-servos table.directions').append('\
        <tr>\
            <td class="name" style="text-align: center">' + name + '</td>\
            <td class="direction" style="text-align: right">\
                <select name="direction">\
                    <option value="0">Normal</option>\
                    <option value="1">Reverse</option>\
                </select>\
            </td>\
        </tr>\
    ');
    
    if (bit_check(SERVO_CONFIG[obj].rate, bitpos)) val = 1;
    else val = 0;

    $('div.tab-servos table.directions tr:last select').val(val);
    $('div.tab-servos table.directions tr:last select').data('info', {'obj': obj, 'bitpos': bitpos});
}

function process_servos(name, alternate, obj, directions) {
    $('div.supported_wrapper').show();
    
    $('div.tab-servos table.fields').append('\
        <tr> \
            <td style="text-align: center">' + name + '</td>\
            <td class="middle"><input type="number" min="1000" max="2000" value="' + SERVO_CONFIG[obj].middle +'" /></td>\
            <td class="min"><input type="number" min="1000" max="2000" value="' + SERVO_CONFIG[obj].min +'" /></td>\
            <td class="max"><input type="number" min="1000" max="2000" value="' + SERVO_CONFIG[obj].max +'" /></td>\
            <td class="channel">\
                <input type="checkbox"/>\
                <input type="checkbox"/>\
                <input type="checkbox"/>\
                <input type="checkbox"/>\
                <input type="checkbox"/>\
                <input type="checkbox"/>\
                <input type="checkbox"/>\
                <input type="checkbox"/>\
            </td>\
            <td class="direction">\
                <input class="first" type="checkbox"/><span class="name">' + name + '</span>\
                <input class="second" type="checkbox"/><span class="alternate">' + alternate + '</span>\
            </td>\
        </tr> \
    ');    
    
    if (SERVO_CONFIG[obj].middle <= 7) {
        $('div.tab-servos table.fields tr:last td.middle input').prop('disabled', true);
        $('div.tab-servos table.fields tr:last td.channel').find('input').eq(SERVO_CONFIG[obj].middle).prop('checked', true);
    }
    
    if (directions == true) {
        $('div.tab-servos table.fields tr:last td.direction input:first').prop('checked', bit_check(SERVO_CONFIG[obj].rate, 0));
        $('div.tab-servos table.fields tr:last td.direction input:last').prop('checked', bit_check(SERVO_CONFIG[obj].rate, 1));
    } else if (directions == 2) {
        // removing checkboxes
        $('div.tab-servos table.fields tr:last td.direction').html('');
        
        // adding select box and generating options
        $('div.tab-servos table.fields tr:last td.direction').append('\
            <select class="rate" name="rate"></select>\
        ');
        
        var select = $('div.tab-servos table.fields tr:last td.direction select');
        
        for (var i = 100; i > -101; i--) {
            select.append('<option value="' + i + '">Rate: ' + i + '%</option>');
        }
        
        // select current rate
        select.val(SERVO_CONFIG[obj].rate);
    } else {
        // removing checkboxes
        $('div.tab-servos table.fields tr:last td.direction').html('');
    }
    
    $('div.tab-servos table.fields tr:last').data('info', {'obj': obj});
    
    // UI hooks
    $('div.tab-servos table.fields tr:last td.channel').find('input').click(function() {
        if($(this).is(':checked')) {
            $(this).parent().parent().find('td.middle input').prop('disabled', true);
            $(this).parent().find('input').not($(this)).prop('checked', false);
        } else {
            $(this).parent().parent().find('td.middle input').prop('disabled', false).val(1500);
        }
    });
}