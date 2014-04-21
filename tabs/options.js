function tab_initialize_options() {
    ga_tracker.sendAppView('Options');

    $('#content').load("./tabs/options.html", function() {
        GUI.active_tab = 'options';

        // translate to user-selected language
        localize();

        if (configuration_received) {
            $('a.back').hide();
        } else {
            $('a.back').click(function() {
                $('#tabs > ul li').removeClass('active'); // de-select any selected tabs
                tab_initialize_default();
            });
        }

        // if notifications are enabled, or wasn't set, check the notifications checkbox
        chrome.storage.local.get('update_notify', function(result) {
            if (typeof result.update_notify === 'undefined' || result.update_notify) {
                $('div.notifications input').prop('checked', true);
            }
        });

        $('div.notifications input').change(function() {
            var check = $(this).is(':checked');

            chrome.storage.local.set({'update_notify': check});
        });

        // if tracking is enabled, check the statistics checkbox
        if (ga_tracking == true) {
            $('div.statistics input').prop('checked', true);
        }

        $('div.statistics input').change(function() {
            var check = $(this).is(':checked');

            ga_tracking = check;

            ga_config.setTrackingPermitted(check);
        });
    });
}