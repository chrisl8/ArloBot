$( document ).ready(function() {

    $('input#say').keypress(function(e) {
        if(e.which == 13) {
            e.preventDefault();
            talkToBot();
        }
    });

    $('button#talk-button').click(function(e){
        e.preventDefault();
        talkToBot();
    });

    var sessionid;

    function talkToBot(){

        $('div.say.bot').html('<img src="images/thinking.gif" />');
        $('button#talk-button').attr('disabled',true);
        $('input#say').attr('disabled',true);
        $('button#status-check').attr('disabled',true);


        var request = $.ajax({
            url: "https://twoflower.ekpyroticfrood.net/chat",
            method: "POST",
            data: { say:$('#say').val(), cid:$('#chat-id').val(), sessionid: sessionid },
            dataType: "json"
        });

        request.done(function( data ) {
            console.log(data);
            if (data.sessionid) {
                sessionid = data.sessionid;
            }
            $('.say.bot').html('&nbsp;');
            $('div.say.bot').html(data.botsay);
            $('input#say').val('');

            $('input#say').removeAttr('disabled');
            $('button#talk-button').removeAttr('disabled');
            $('button#status-check').removeAttr('disabled');
	    $('input#say').focus();

        });

        request.fail(function( jqXHR, textStatus ) {
            console.log(`Failed request: ${textStatus}`);
            $('.say.bot').html('&nbsp;');
            $('div.say.bot').html("Sorry, I was daydreaming. Could you repeat that?");
            $('input#say').val('');

            $('input#say').removeAttr('disabled');
            $('button#talk-button').removeAttr('disabled');
            $('button#status-check').removeAttr('disabled');
	    $('input#say').focus();
        });
    }

});

