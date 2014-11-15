function update_sortable_filters() {
    $.each($(".filtergroup-body"), function(index, filtergroup) {
        if($(filtergroup).find(".filter").length == 1) {
            $(filtergroup).find(".filter").addClass("sort-disabled");
        } else {
            $(filtergroup).find(".filter").removeClass("sort-disabled");
        }
    });

    $(".filtergroup-body").sortable("refresh");
}

function create_sortable_filters() {
    $(".filtergroup-body").sortable({
        items: ".filter",
        cancel: ".sort-disabled .ep",
        connectWith: ".filtergroup-body",
        stop: function(event, ui) {
            update_sortable_filters();
        }
    });
}

function draw_image(canvas_id, imgString){
    var canvas = $("#" + canvas_id)[0];
    var ctx = canvas.getContext("2d");
    var image = new Image();

    if(imgString != "None") {
        image.src = ((imgString != null) ? "data:image/jpg;base64," + imgString : null);

    } else {
        image.src = "images/no_feed.png";
    }

    image.onload = function() {
        // FIXME: Auto adjust canvas dimensions
        /*canvas.style.width='100%';
        canvas.style.height='100%';
        canvas.width  = canvas.offsetWidth;
        canvas.height = canvas.offsetHeight;*/
        ctx.drawImage(image, 0, 0, canvas.width, canvas.height);
    };
}

function build_topic_list(topic_selector_id, data) {
    $("#" + topic_selector_id).empty();
    $("#" + topic_selector_id).append('<option value="None">None</option>');

    data = JSON.parse(data);
    topics = data.topics;
    selected = data.selected;

    $.each(topics, function(i, optgroups) {
        $.each(optgroups, function(groupName, options) {
            var $optgroup = $("<optgroup>", {label: groupName});
            $optgroup.appendTo($("#" + topic_selector_id));

            $.each(options, function(j, option) {
                var $option = $("<option>", {text: option, value: groupName});
                $option.appendTo($optgroup);
            });
        });
    });

    $("#" + topic_selector_id + " option").filter(function () { return $(this).html() == selected; }).val();
}

jsPlumb.ready(function() {
	var instance = jsPlumb.getInstance({
		Endpoint : ["Dot", {radius:2}],
		HoverPaintStyle : {strokeStyle:"#1e8151", lineWidth:2 },
		ConnectionOverlays : [
			[ "Arrow", {
				location:1,
				id:"arrow",
                length:14,
                foldback:0.8
			} ],
            [ "Label", { label:"FOO", id:"label", cssClass:"aLabel" }]
		],
		Container:"roscvm-workspace"
	});

    window.jsp = instance;

	instance.doWhileSuspended(function() {
		var isFilterSupported = instance.isDragFilterSupported();

        var eps = jsPlumb.getSelector(".ep");
        for (var i = 0; i < eps.length; i++) {
            var e = eps[i], p = e.parentNode;
            instance.makeSource(e, {
                parent:p,
                anchor:"Continuous",
                connector:[ "StateMachine", { curviness:20 } ],
                connectorStyle:{ strokeStyle:"#5c96bc",lineWidth:2, outlineColor:"transparent", outlineWidth:4 },
                maxConnections:5,
                onMaxConnections:function(info, e) {
                    alert("Maximum connections (" + info.maxConnections + ") reached");
                }
            });
        }
	});

	instance.makeTarget(jsPlumb.getSelector(".filter"), {
		anchor:"Continuous",
		allowLoopback:true
	});
});

$(document).ready(function () {
    var input1 = new WebSocket("ws://localhost:8888/input1/");
    var topics1 = new WebSocket("ws://localhost:8888/input1/topic/");
    var input2 = new WebSocket("ws://localhost:8888/input2/");
    var topics2 = new WebSocket("ws://localhost:8888/input1/topic/");

    draw_image("feed1", "None");
    draw_image("feed2", "None");

    input1.onmessage = function(evt) {
        draw_image("feed1", evt.data);
    };

    topics1.onmessage = function(evt) {
        build_topic_list("feed1-topics", evt.data);
    };

    input2.onmessage = function(evt) {
        draw_image("feed2", evt.data);
    };

    topics2.onmessage = function(evt) {
        build_topic_list("feed2-topics", evt.data);
    };

    $("#feed1-topics").change(function () {
        input1.send([$("option:selected", this).text(), $("option:selected", this).val()])
    });

    $("#feed2-topics").change(function () {
        input2.send([$("option:selected", this).text(), $("option:selected", this).val()])
    });

    create_sortable_filters();

    $(".filtergroup-add.top").click(function() {
        $(this).after('<div class="filtergroup"><div class="filtergroup-header"></div><dl class="filtergroup-body"><dd class="filter-add left sort-disabled">◁ Add</dd><dd class="filter sort-disabled"><div class="ep"></div></dd><dd class="filter-add right sort-disabled">Add ▷</dd></dl></div></div>');

        create_sortable_filters();
    });

    $(".filtergroup-add.bottom").click(function() {
        $(this).before('<div class="filtergroup"><div class="filtergroup-header"></div><dl class="filtergroup-body"><dd class="filter-add left sort-disabled">◁ Add</dd><dd class="filter sort-disabled"><div class="ep"></div></dd><dd class="filter-add right sort-disabled">Add ▷</dd></dl></div></div>');

        create_sortable_filters();
    });

    $("body").on("click", ".filter-add.right", function() {
        $(this).before('<dd class="filter"><div class="ep"></div></dd>');
        update_sortable_filters();
    });

    $("body").on("click", ".filter-add.left", function() {
        $(this).after('<dd class="filter"><div class="ep"></div></dd>');
        update_sortable_filters();
    });
});