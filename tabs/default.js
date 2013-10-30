function tab_initialize_default() {
    $('#content').load("./tabs/default.html", function() {
        // load changelog content
        $('div.changelog.configurator .wrapper').load('./changelog.html');
    });
}