function tab_initialize_default() {
    $('#content').load("./tabs/default.html", function() {
        GUI.active_tab = 'default';
        
        // handle smaller resolution
        if (screen.height <= 600) {
            $('p.created').hide();
        }
        
        // load changelog content
        $('div.changelog.configurator .wrapper').load('./changelog.html');
        
        // UI Hooks
        $('a.firmware_flasher').click(function() {
            tab_initialize_firmware_flasher();
        });
    });
}