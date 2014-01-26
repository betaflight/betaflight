function tab_initialize_motor_outputs() {
    ga_tracker.sendAppView('Motor Outputs Page');
    
    $('#content').load("./tabs/motor_outputs.html", function() {
        GUI.active_tab = 'motor_outputs';
        
        // if CAP_DYNBALANCE is true
        if (bit_check(CONFIG.capability, 2)) {
            $('div.motor_testing').show();
        }
        
        send_message(MSP_codes.MSP_MISC, MSP_codes.MSP_MISC, false, function() {
            $('input.min').val(MISC.minthrottle);
            $('input.max').val(MISC.maxthrottle);
            
            
            $('div.sliders input').prop('min', MISC.minthrottle);
            $('div.sliders input').prop('max', MISC.maxthrottle);
            $('div.sliders input').val(MISC.minthrottle);
            $('div.values li:not(:last)').html(MISC.minthrottle);
        });
        
        // UI hooks
        $('div.sliders input:not(.master)').change(function() {
            var index = $(this).index();
            
            $('div.values li').eq(index).html($(this).val());
            
            // send data to mcu
            var buffer_out = [];
            
            for (var i = 0; i < 8; i++) {
                var val = parseInt($('div.sliders input').eq(i).val());
                
                buffer_out.push(lowByte(val));
                buffer_out.push(highByte(val));
            }
            
            send_message(MSP_codes.MSP_SET_MOTOR, buffer_out);
        });
        
        $('div.sliders input.master').change(function() {
            var val = $(this).val();
            
            $('div.sliders input').val(val);
            $('div.sliders input:not(:last):first').change();
        });
        
        $('div.notice input[type="checkbox"]').change(function() {
            if ($(this).is(':checked')) {
                $('div.sliders input, .notice input[type="number"]').prop('disabled', false);
            } else {
                // disable sliders / min max
                $('div.sliders input, .notice input[type="number"]').prop('disabled', true);
                
                // change all values to default
                $('div.sliders input').val(1000);
                $('div.values li:not(:last)').html(1000);
                
                // trigger change event so values are sent to mcu
                $('div.sliders input').change();
            }
        });
        
        $('div.notice input[type="number"]').change(function() {
            var min = parseInt($('div.notice .min').val());
            var max = parseInt($('div.notice .max').val());
            
            
            $('div.sliders input').prop('min', min);
            $('div.sliders input').prop('max', max);
            
            // trigger change event so values are sent to mcu
            $('div.sliders input').change();
        });
        
        // enable Motor data pulling
        GUI.interval_add('motor_poll', function() {
            // Request New data
            send_message(MSP_codes.MSP_STATUS, MSP_codes.MSP_STATUS, false, function() {
                send_message(MSP_codes.MSP_MOTOR, MSP_codes.MSP_MOTOR, false, function() {
                    send_message(MSP_codes.MSP_SERVO, MSP_codes.MSP_SERVO, false, function() {
                        // Update UI
                        for (var i = 0; i < MOTOR_DATA.length; i++) {
                            MOTOR_DATA[i] -= 1000; 
                            var margin_top = 220.0 - (MOTOR_DATA[i] * 0.22);
                            var height = (MOTOR_DATA[i] * 0.22);
                            var color = parseInt(MOTOR_DATA[i] * 0.256);
                            $('.motor-' + i + ' .indicator').css({'margin-top' : margin_top + 'px', 'height' : height + 'px', 'background-color' : 'rgb(' + color + ',0,0)'});
                        }
                        
                        for (var i = 0; i < SERVO_DATA.length; i++) {
                            SERVO_DATA[i] -= 1000; 
                            var margin_top = 220.0 - (SERVO_DATA[i] * 0.22);
                            var height = (SERVO_DATA[i] * 0.22);
                            var color = parseInt(SERVO_DATA[i] * 0.256);
                            $('.servo-' + i + ' .indicator').css({'margin-top' : margin_top + 'px', 'height' : height + 'px', 'background-color' : 'rgb(' + color + ',0,0)'});
                        } 
                    });
                });
            });
        }, 50, true);
    });
}