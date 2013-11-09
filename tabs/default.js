function tab_initialize_default() {
    $('#content').load("./tabs/default.html", function() {
        GUI.active_tab = 'default';
        
        // load changelog content
        $('div.changelog.configurator .wrapper').load('./changelog.html');
    });
}