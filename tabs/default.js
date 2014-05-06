function tab_initialize_default() {
    GUI.active_tab = 'default';

    $('#content').load("./tabs/default.html", function() {
        // translate to user-selected language
        localize();

        // load changelog content
        $('div.changelog.configurator .wrapper').load('./changelog.html');

        // UI Hooks
        $('a.firmware_flasher').click(tab_initialize_firmware_flasher);
    });
}