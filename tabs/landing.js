'use strict';

TABS.landing = {};
TABS.landing.initialize = function (callback) {
    var self = this;

    if (GUI.active_tab != 'landing') {
        GUI.active_tab = 'landing';
        googleAnalytics.sendAppView('Landing');
    }

    $('#content').load("./tabs/landing.html", function () {
        // translate to user-selected language
        localize();

        // load changelog content
        $('div.changelog.configurator .changewrapper').load('./changelog.html');

        $('div.welcome a, div.sponsors a').click(function () {
            googleAnalytics.sendEvent('ExternalUrls', 'Click', $(this).prop('href'));
        });
        

/** changelog trigger **/
$("#changelog_button").on('click', function() {
    var state = $(this).data('state2');
    if ( state ) {
        $(".changelog").animate({width: 0}, 200);
	    $(".changewrapper").animate({opacity: 0}, 500);
		$(".changewrapper").removeClass('active');

        state = false;
    }else{
        $(".changelog").animate({width: 220}, 200);
        $(".changewrapper").animate({opacity: 1}, 500);
        $(".changewrapper").addClass('active');

        state = true;
    }
    $(this).text(state ? 'Close' : 'Changelog');
    $(this).data('state2', state);
    
});




        GUI.content_ready(callback);
    });
};

TABS.landing.cleanup = function (callback) {
    if (callback) callback();
};