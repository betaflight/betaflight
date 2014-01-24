function tab_initialize_receiver() {
    ga_tracker.sendAppView('Receiver Page');
    
    $('#content').load("./tabs/receiver.html", function() {
        GUI.active_tab = 'receiver';
        
        send_message(MSP_codes.MSP_RC_TUNING, MSP_codes.MSP_RC_TUNING, false, function() {
            // fill in data from RC_tuning
            $('.tunings .throttle input[name="mid"]').val(RC_tuning.throttle_MID.toFixed(2));
            $('.tunings .throttle input[name="expo"]').val(RC_tuning.throttle_EXPO.toFixed(2));

            $('.tunings .rate input[name="rate"]').val(RC_tuning.RC_RATE.toFixed(2));
            $('.tunings .rate input[name="expo"]').val(RC_tuning.RC_EXPO.toFixed(2));
            
            // Setup plot variables and plot it self
            var samples_i = 300;
            
            RX_plot_data = new Array(8);
            for (var i = 0; i < 8; i++) {
                RX_plot_data[i] = new Array();
            }    

            for (var i = 0; i <= 300; i++) {
                RX_plot_data[0].push([i, 0]);
                RX_plot_data[1].push([i, 0]);
                RX_plot_data[2].push([i, 0]);
                RX_plot_data[3].push([i, 0]);
                RX_plot_data[4].push([i, 0]); 
                RX_plot_data[5].push([i, 0]); 
                RX_plot_data[6].push([i, 0]); 
                RX_plot_data[7].push([i, 0]); 
            }
            
            e_RX_plot = document.getElementById("RX_plot");
            
            RX_plot_options = {
                title: "Channel width (us)",
                shadowSize: 0,
                yaxis : {
                    max: 2200,
                    min: 800,
                    noTicks: 10
                },
                xaxis : {
                    //noTicks = 0
                },
                grid : {
                    backgroundColor: "#FFFFFF"
                },
                legend : {
                    position: "we",
                    backgroundOpacity: 0
                }        
            }; 
            
            // UI Hooks
            
            // curves    
            $('.tunings .throttle input').change(function() {
                setTimeout(function() {
                    var mid = parseFloat($('.tunings .throttle input[name="mid"]').val());
                    var expo = parseFloat($('.tunings .throttle input[name="expo"]').val());
                    
                    var throttle_curve = $('.throttle_curve canvas').get(0);
                    var context = throttle_curve.getContext("2d");
                    context.clearRect(0, 0, 220, 58);
                    
                    // math magic by englishman
                    var midx = 220 * mid;
                    var midxl = midx * .5;
                    var midxr = (((220 - midx) * .5) + midx);
                    var midy = 58 - (midx * (58 / 220));
                    var midyl = 58 - ((58 - midy) * .5 *(expo + 1));
                    var midyr = (midy / 2) * (expo + 1);        
                    
                    context.beginPath();
                    context.moveTo(0, 58);
                    context.quadraticCurveTo(midxl, midyl, midx, midy);
                    context.moveTo(midx, midy);
                    context.quadraticCurveTo(midxr, midyr, 220, 0);     
                    
                    context.lineWidth = 2;
                    context.stroke();
                }, 0); // race condition, that should always trigger after all events are processed
            }).change();
            
            $('.tunings .rate input').change(function() {
                setTimeout(function() {
                    var rate = parseFloat($('.tunings .rate input[name="rate"]').val());
                    var expo = parseFloat($('.tunings .rate input[name="expo"]').val());  
                    
                    var pitch_roll_curve = $('.pitch_roll_curve canvas').get(0);
                    var context = pitch_roll_curve.getContext("2d");
                    context.clearRect(0, 0, 220, 58);
                    
                    // math magic by englishman
                    var ratey = 58 * rate;
                    
                    context.beginPath();
                    context.moveTo(0, 58);
                    context.quadraticCurveTo(110, 58 - ((ratey / 2) * (1 - expo)), 220, 58 - ratey);
                    context.lineWidth = 2;
                    context.stroke();
                }, 0); // race condition, that should always trigger after all events are processed
            }).change();
            
            $('a.refresh').click(function() {
                send_message(MSP_codes.MSP_RC_TUNING, MSP_codes.MSP_RC_TUNING, false, function() {
                    // fill in data from RC_tuning
                    $('.tunings .throttle input[name="mid"]').val(RC_tuning.throttle_MID.toFixed(2));
                    $('.tunings .throttle input[name="expo"]').val(RC_tuning.throttle_EXPO.toFixed(2));

                    $('.tunings .rate input[name="rate"]').val(RC_tuning.RC_RATE.toFixed(2));
                    $('.tunings .rate input[name="expo"]').val(RC_tuning.RC_EXPO.toFixed(2));
                    
                    // update visual representation
                    $('.tunings .throttle input').change();
                    $('.tunings .rate input').change();
                });
            });
            
            $('a.update').click(function() {
                // catch RC_tuning changes
                RC_tuning.throttle_MID = parseFloat($('.tunings .throttle input[name="mid"]').val());
                RC_tuning.throttle_EXPO = parseFloat($('.tunings .throttle input[name="expo"]').val());
                
                RC_tuning.RC_RATE = parseFloat($('.tunings .rate input[name="rate"]').val());
                RC_tuning.RC_EXPO = parseFloat($('.tunings .rate input[name="expo"]').val());              
                
                var RC_tuning_buffer_out = new Array();
                RC_tuning_buffer_out[0] = parseInt(RC_tuning.RC_RATE * 100);
                RC_tuning_buffer_out[1] = parseInt(RC_tuning.RC_EXPO * 100);
                RC_tuning_buffer_out[2] = parseInt(RC_tuning.roll_pitch_rate * 100);
                RC_tuning_buffer_out[3] = parseInt(RC_tuning.yaw_rate * 100);
                RC_tuning_buffer_out[4] = parseInt(RC_tuning.dynamic_THR_PID * 100);
                RC_tuning_buffer_out[5] = parseInt(RC_tuning.throttle_MID * 100);
                RC_tuning_buffer_out[6] = parseInt(RC_tuning.throttle_EXPO * 100);
                
                // Send over the RC_tuning changes
                send_message(MSP_codes.MSP_SET_RC_TUNING, RC_tuning_buffer_out);        

                // Save changes to EEPROM
                send_message(MSP_codes.MSP_EEPROM_WRITE, MSP_codes.MSP_EEPROM_WRITE, false, function() {
                    var element = $('a.update');
                    element.addClass('success');
                    
                    GUI.timeout_add('success_highlight', function() {
                        element.removeClass('success');
                    }, 2000);
                });
            });
            
            // enable RC data pulling
            GUI.interval_add('receiver_poll', function() {
                // Update UI with latest data
                $('.tab-receiver meter:eq(0)').val(RC.throttle);
                $('.tab-receiver .value:eq(0)').html(RC.throttle);
                
                $('.tab-receiver meter:eq(1)').val(RC.pitch);
                $('.tab-receiver .value:eq(1)').html(RC.pitch);
                
                $('.tab-receiver meter:eq(2)').val(RC.roll);
                $('.tab-receiver .value:eq(2)').html(RC.roll);
                
                $('.tab-receiver meter:eq(3)').val(RC.yaw);
                $('.tab-receiver .value:eq(3)').html(RC.yaw);
                
                
                $('.tab-receiver meter:eq(4)').val(RC.AUX1);
                $('.tab-receiver .value:eq(4)').html(RC.AUX1);
                
                $('.tab-receiver meter:eq(5)').val(RC.AUX2);
                $('.tab-receiver .value:eq(5)').html(RC.AUX2);
                
                $('.tab-receiver meter:eq(6)').val(RC.AUX3);
                $('.tab-receiver .value:eq(6)').html(RC.AUX3);
                
                $('.tab-receiver meter:eq(7)').val(RC.AUX4);
                $('.tab-receiver .value:eq(7)').html(RC.AUX4);
                
                // push latest data to the main array
                RX_plot_data[0].push([samples_i, RC.throttle]);
                RX_plot_data[1].push([samples_i, RC.pitch]);
                RX_plot_data[2].push([samples_i, RC.roll]);
                RX_plot_data[3].push([samples_i, RC.yaw]);
                RX_plot_data[4].push([samples_i, RC.AUX1]);
                RX_plot_data[5].push([samples_i, RC.AUX2]);
                RX_plot_data[6].push([samples_i, RC.AUX3]);
                RX_plot_data[7].push([samples_i, RC.AUX4]);        

                // Remove old data from array
                while (RX_plot_data[0].length > 300) {
                    RX_plot_data[0].shift();
                    RX_plot_data[1].shift();
                    RX_plot_data[2].shift();
                    RX_plot_data[3].shift();
                    RX_plot_data[4].shift();
                    RX_plot_data[5].shift();
                    RX_plot_data[6].shift();
                    RX_plot_data[7].shift();
                };
                
                // redraw plot
                Flotr.draw(e_RX_plot, [
                    {data: RX_plot_data[0], label: "THROTTLE"}, 
                    {data: RX_plot_data[1], label: "PITCH"},
                    {data: RX_plot_data[2], label: "ROLL"},
                    {data: RX_plot_data[3], label: "YAW"},
                    {data: RX_plot_data[4], label: "AUX1"},
                    {data: RX_plot_data[5], label: "AUX2"},
                    {data: RX_plot_data[6], label: "AUX3"},
                    {data: RX_plot_data[7], label: "AUX4"} ], RX_plot_options);
                    
                samples_i++;
                
                // Request new data
                send_message(MSP_codes.MSP_STATUS, MSP_codes.MSP_STATUS);
                send_message(MSP_codes.MSP_RC, MSP_codes.MSP_RC);
            }, 50);
        });
    });
}